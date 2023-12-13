#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define buffSize 256 
#define dump_car_drive_events 1 // debug util 0 for no comment 1 for debug

//generic parameters
#define road_length 500  // road length in meters
#define road_lanes_count 6 // road lanes count in both directions
#define road_lane_width  3  // road lane width in meters
#define car_length  5    //average car length in meters  
#define mid_car_separation  1    //average separation between car in meters while standing at red light 
#define driver_reaction_time 2   // average driver reaction time in secods, i.e. duration between preceeding car movement is noticed and car behind it start accelerating
#define car_max_speed        12  // average car speed in m/s , after acelarating car continues to move uniformly at this speed until stop requested
#define car_acceleration_rate  2 // average accelaration rate in m/s^2  till car reachs car_max_speed



#define time_till_unifiorm_speed_achieved  (car_max_speed/ car_acceleration_rate)   // a = (v_end - v_start)/dt  => dt = v_end/a  ( as v_start is 0)
#define total_simulation_duration 3600


// simple mode parameters (timer-controlled traffic lights)
#define simple_mode_green_light_duration 30  // duration in seconds of green light for cars
#define simple_mode_red_light_duration 20  // duration in seconds of red light for cars ( aka  green light for pedestrians)


// smart mode parameters (push-button controlled traffic lights)
#define smart_mode_distribution_file_path "RoadRegister.csv"  // source of actual chronological pedestrians arrivals
#define pedestrian_walk_speed    1.3   // avg pedestrian walk speed
#define pedestrian_road_cross_duration   (road_lanes_count * road_lane_width / pedestrian_walk_speed)   // avg time required to cross the road by pedestrian 
#define pedestrian_reaction_time 5   // average time since pedestrian arrival till he/she pushes the button after situation analyses
#define smart_mode_green_switch_delay 30 // fixed delay between traffic light switches to green after button is pushed
#define smart_mode_green_duration 20 // duration of green light for pedestrians

///////////////////////////////

struct result_struct{
    
    int car_count;
    int total_green_light_time;
    int total_light_switch_count;

};


// function estimates the distance in meters, car will pass in given time period, 
// assuming that initial/starting speed is 0, and final speed in car_max_speed
// after which car stops accelaration and starts to move uniformly
int evaluate_car_passed_distance(int period)
{
    if(period <= 0) 
        return 0;

    int distance_passed_while_accelerating = 0;
    int distance_passed_in_uniform_movement = 0;
    int acceleration_time = period <= time_till_unifiorm_speed_achieved ? period :  time_till_unifiorm_speed_achieved;
    
    // S = a * t^2 / 2
    distance_passed_while_accelerating = car_acceleration_rate * acceleration_time * acceleration_time / 2;
    
    if(period > acceleration_time)
    { // S = v * t
        period -= acceleration_time;
        distance_passed_in_uniform_movement = car_max_speed * period;
    }

    return distance_passed_while_accelerating + distance_passed_in_uniform_movement;
}

// returns count of car which has passed traffic light
int run_simulation_simple_mode_iteration(int time_tick, int green_period, struct result_struct *ptr)
{    

    int current_car_position = 0;
    int passed_car_count = 0;
    int current_car_travel_distance_to_leave = 0;
    int car_actual_distance_in_given_time = 0;
    int remaing_duration = green_period;
    while (remaing_duration > 0)
    {
        current_car_travel_distance_to_leave = current_car_position + car_length;
        car_actual_distance_in_given_time = evaluate_car_passed_distance(remaing_duration);
        if(car_actual_distance_in_given_time < current_car_travel_distance_to_leave)
        {
            break;
        }

        passed_car_count++;
        current_car_position += car_length + mid_car_separation;
        
        if(current_car_position > road_length){// required to niglate the effect of long arrival time out of the road, thi spart is out of the scope
            current_car_position = road_length;
        }

        remaing_duration -= driver_reaction_time;
    }

    if(dump_car_drive_events && passed_car_count> 0)
        printf("TimeOffset: %d, Duration: %d, Passed Cars: %d\n", time_tick, green_period, passed_car_count);

    ptr->car_count += road_lanes_count * passed_car_count;
    ptr->total_green_light_time += green_period;
    ptr->total_light_switch_count++;

    return  passed_car_count;
}


// at time point 0, we assume that road is fully filled with car , first car is waiting at traffic light
int run_simulation_simple_mode(struct result_struct *ptr)
{        
    int time_tick = 0;
       
    while(time_tick <= total_simulation_duration)
    {
        // green 
        run_simulation_simple_mode_iteration(time_tick, simple_mode_green_light_duration, ptr);
        time_tick += simple_mode_green_light_duration + simple_mode_red_light_duration;
        //red:  fit remaining cars, first cars will be pushed to position 0, capacity will be filled
    }
   
    return 1; 
}

/*
returns:
 -1 : Bad input file
 -2 : Input file contains event at time point exceeding simulation duration
 -3: Input file events are not given in chronological order
*/

int run_simulation_smart_mode(struct result_struct *ptr)
{

    char Buff[buffSize];
    FILE* fptr = fopen(smart_mode_distribution_file_path, "r");
    if(fptr == NULL){        
        return -1;
    }

    if(fgets(Buff, buffSize, fptr) == NULL) //skipping 1st/header/ line
        return -1;
        
    int car_sim_time_offset = 0;
    
    int prev_rec_time_offset = 0;
    int rec_time_offset;
    int rec_pedestrians_count;

    int pedestrian_green_start_offset = -1;
    int pedestrian_green_end_offset = -1;

    int queue_count = 0;

    while ( fgets(Buff, buffSize, fptr) != NULL){
        
        rec_time_offset = atoi(strtok(Buff,","));
        rec_pedestrians_count = atoi(strtok(0,","));
        
        if( rec_time_offset > total_simulation_duration)
            return -2;

        if(rec_time_offset <  prev_rec_time_offset)
            return -3;

        prev_rec_time_offset = rec_time_offset;

        if(pedestrian_green_start_offset < 0)
        {
            pedestrian_green_start_offset = rec_time_offset + pedestrian_reaction_time + smart_mode_green_switch_delay;
            pedestrian_green_end_offset = pedestrian_green_start_offset + smart_mode_green_duration;
           
            continue;
        }
            
        if(rec_time_offset + pedestrian_road_cross_duration <= pedestrian_green_end_offset)
        {            
            continue;
        }


        // cars to pass before pedestrian_green_start_offset of prev_rec
        if(car_sim_time_offset < pedestrian_green_start_offset)
        {            
            run_simulation_simple_mode_iteration(car_sim_time_offset, pedestrian_green_start_offset - car_sim_time_offset, ptr);
            car_sim_time_offset = pedestrian_green_end_offset;
        }
          
              
        // set next green light time
               
        pedestrian_green_start_offset = rec_time_offset + pedestrian_reaction_time + smart_mode_green_switch_delay;
        pedestrian_green_end_offset = pedestrian_green_start_offset + smart_mode_green_duration;
        
    }

    fclose(fptr);
    // flush event for last pedestrian
    if(car_sim_time_offset < pedestrian_green_start_offset)
    {
        run_simulation_simple_mode_iteration(car_sim_time_offset, pedestrian_green_start_offset - car_sim_time_offset, ptr);
    }

    // run cars till end of simulation, no more pedestrians and traffic light stays green till the end
    if(pedestrian_green_end_offset < total_simulation_duration)
    {
        run_simulation_simple_mode_iteration(pedestrian_green_end_offset, total_simulation_duration - pedestrian_green_end_offset, ptr);
    }


    return 1;
}

int report_smart_mode_error(int result)
{
    if(result > 0)
    return 0;
 
    switch(result)
    {
        case -1: 
             printf("Error: Bad input file");
             break;   
        case -2: 
             printf("Error: Input file contains event at time point exceeding simulation duration");
             break;   

        case -3: 
             printf("Error: Input file events should be given in a chronological order");
             break;   


        case 0: 
             printf("Error: Unexpected error due to bad input, simulation result equals to 0.");
             break;   
   
        default: 
            printf("Error: Unexpected error.");
            break;   
    }

    return 1;
}

int main(){
    
    char ch;
    int result = 0;
    int input_state = 0;

    struct result_struct *ptr = malloc(sizeof(struct result_struct));
    ptr->car_count = 0;
    ptr->total_green_light_time = 0;
    ptr->total_light_switch_count = 0;

    while (1)
    {
        input_state = 0;
        printf("Select Simulation Mode: \n");
        printf("1: Simple simulation (fixed green/red timed-controlled switching).\n");
        printf("2: Smart simulation (red switching on demand).\n");
        printf("3: Exit\n");
        printf("Enter your choice of operations: ");
        scanf("%c", &ch);
        switch (ch)
        {
            case '1':
                result = run_simulation_simple_mode(ptr);
                input_state = 1;
            break;

            case '2':
                result  = run_simulation_smart_mode(ptr);
                input_state = 1;
            break;
            
            case '3':
                input_state = 2;
                
            default:
                printf("Incorrect choice \n");
              
        } 

        if(input_state == 0) continue;

        if(input_state == 1 && !report_smart_mode_error(result))
        {
            printf("=====================================================\n");
            printf("Total car count passed crossing in given mode: %d, \n"
                   "With total car green light duration: %d and "
                   "total traffic light switch count: %d.\n", 
                                                            ptr->car_count, 
                                                            ptr-> total_green_light_time,
                                                            ptr->total_light_switch_count);
            printf("=====================================================\n\n");
                
        }

        if(ptr != NULL)
        {
            free(ptr);
        }

        exit(0);

    } 


}
