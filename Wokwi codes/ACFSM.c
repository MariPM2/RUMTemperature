#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

char* current_state = "off";
int current_temp = 0;

typedef struct {
    char* name;
} State;

typedef struct {
    char* name;
} Event;

typedef struct {
    char* current_state;
    char* current_event;
    char* next_state;
    void (*handle_event)();
    void (*handle_exit)();
} Transition;

void handle_event() { 
    printf("Entering state: %s \n", current_state); 
}

void handle_exit() { 
    printf("Exiting state: %s \n", current_state); 
}

State AC_states[] = {
  {"on"},
  {"off"}
};

Event AC_events[] = {
  {"turn on"},
  {"turn off"},
  {"increase temp"},
  {"decrease temp"}
};

Transition AC_transitions[] = {
  {"on", "turn on", "on", handle_event, handle_exit},
  {"on", "increase temp", "on", handle_event, handle_exit},
  {"on", "decrease temp", "on", handle_event, handle_exit},
  {"on", "turn off", "off", handle_event, handle_exit},
  {"off", "turn on", "on", handle_event, handle_exit},
  {"off", "turn off", "off", handle_event, handle_exit},
};

void init_fsm(){
    current_state = "off";
    current_temp = 70;
}

void handle_fsm_event(char * event, char * state, int * temp){
    
    for(int i=0; i<sizeof(AC_transitions)/sizeof(AC_transitions[0]); i++){
        Transition transition = AC_transitions[i];
        if(strcmp(transition.current_state, state) == 0 && strcmp(transition.current_event, event) == 0){
            transition.handle_exit();
            current_state = transition.next_state;
            transition.handle_event();
            break;
        }
    }

    if(strcmp(current_state, AC_states[0].name /*"on"*/) == 0){
        if(strcmp(event, AC_events[2].name /*"increase temp"*/) == 0){
            if(*temp < 90){
                (*temp)++;
            }
        }
        else if(strcmp(event, AC_events[3].name /*"decrease temp"*/) == 0){
            if(*temp > 50){
                (*temp)--;
            }
        }
    }

    if(strcmp(current_state, AC_states[1].name /*"off"*/) == 0){ (*temp) = 0; }
}

void fsm_tasks(void *) {
    char * event;

    printf("Initial State: %s \n\n", current_state);
    
    // event = "turn on";
    event = AC_events[0].name;

    printf("Event: %s\n", event);
    handle_fsm_event(event, current_state, &current_temp);
    printf("Initial temp: %d degrees \n\n", current_temp);

    // event = "increase temp";
    event = AC_events[2].name;

    printf("Event: %s\n", event);
    handle_fsm_event(event, current_state, &current_temp);
    printf("Temp: %d degrees \n\n", current_temp);

    printf("Event: %s\n", event);
    handle_fsm_event(event, current_state, &current_temp);
    printf("Temp: %d degrees \n\n", current_temp);

    // event = "decrease temp";
    event = AC_events[3].name;

    printf("Event: %s\n", event);
    handle_fsm_event(event, current_state, &current_temp);
    printf("Temp: %d degrees \n\n", current_temp);

    // event = "turn off";
    event = AC_events[1].name;
    
    printf("Event: %s\n", event);
    handle_fsm_event(event, current_state, &current_temp);
    printf("Temp: %d degrees \n\n", current_temp);

    vTaskDelete(NULL);
}

extern "C" void app_main() {

  init_fsm();

  xTaskCreate(&fsm_tasks, "fsm_tasks", 3000, NULL, 10, NULL);

}
