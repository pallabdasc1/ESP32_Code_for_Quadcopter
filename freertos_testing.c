#include "Arduino.h"

// Task 1 function
void Task1(void *pvParameters) {
    while (1) {
        Serial.println("Task 1 running");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    }
}

// Task 2 function
void Task2(void *pvParameters) {
    while (1) {
        Serial.println("Task 2 running");
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 2 seconds
    }
}

void setup() {
    Serial.begin(115200);
    
    // Create two FreeRTOS tasks
    xTaskCreate(Task1, "Task1", 1000, NULL, 1, NULL);
    xTaskCreate(Task2, "Task2", 1000, NULL, 1, NULL);
}

void loop() {
    // Nothing here, tasks run in parallel
}
