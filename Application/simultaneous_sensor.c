#include "simultaneous_sensor.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/display/Display.h>

#define TASK_PRIORITY                     1

#ifndef TASK_STACK_SIZE
#define TASK_STACK_SIZE                   1024
#endif

typedef struct
{
    uint8_t event;
    void *data;
} ApplicationEvent;

Task_Struct task;

uint8_t task_stack[TASK_STACK_SIZE];

Display_Handle display = NULL;

static void initialise(void);
static void task_fn(UArg argument_one, UArg argument_two);

void simultaneous_sensor(void)
{
    Task_Params task_parameters;

    Task_Params_init(&task_parameters);
    task_parameters.stack = task_stack;
    task_parameters.stackSize = TASK_STACK_SIZE;
    task_parameters.priority = TASK_PRIORITY;

    Task_construct(&task, task_fn, &task_parameters, NULL);
}

static void task_fn(UArg argument_one, UArg argument_two)
{
    initialise();
    for (;;)
    {

    }
}

static void initialise(void)
{
    display = Display_open(Display_Type_UART, NULL);
}
