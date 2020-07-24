#include <Search.h>

void Search::loop()
{
    // Serial1.printf("\tSearch state: %d\n", state);
    switch (state)
    {
    case 1: // spherical search
        update_state(search(MAX_DISTANCE, direction));
        break;
    case 2: // alignment
        update_state(align());
        break;
    case 3: // pickup cans
        pick_up_can(false);
        delay(300);
        update_state(true);
        break;
    case 4: // new direction
        // Serial1.printf("New Direction\n");
        update_state(new_direction());
        break;
    case 5: // linear search
        update_state(linear_search(MAX_DISTANCE));
        // Serial1.printf("Linear Search\n");
        break;
    }
}

void Search::update_state(bool result)
{
    switch (state)
    {
    case 1:
        if (result)
        {
            state = 2;
            break;
        }
        else
        {
            search_fail += 1;
            // Serial1.printf("\t\tFail: %d\n", search_fail);
        }
        if (search_fail > max_search_fail)
        {
            search_fail = 0;
            // direction *= -1;
            direction *= 1;
            state = 4;
        }
        break;
    case 2:
        if (result)
        {
            state = 3;
            break;
        }
        else
        {
            align_fail += 1;
        }
        if (align_fail > max_align_fail)
        {
            align_fail = 0;
            state = 1;
        }
        break;
    case 3:
        state = 1;
        break;
    case 4:
        state = 5;
        break;
    case 5:
        if (result)
        {
            state = 2;
            break;
        }
        else
        {
            linear_search_counter -= 1;
        }
        if (linear_search_counter > 0)
        {
            state = 5;
        }
        else
        {
            linear_search_counter = max_linear_search_fail;
            state = 1;
        }
        break;
    }
}

void Search::enter_arena()
{
    int cm2 = sonar2.ping_median(3) / 57; // take distance measurement to see if it is a bin
    int cm = sonar.ping_cm();             // take distance measurement

    if (cm2 == 0)
    {
        cm2 = 200;
    }
    if (cm < TARGET_DISTANCE && cm != 0 && cm2 > cm * 2 && cm2 > 40)
    {
        left_motor.run_motor(0);
        right_motor.run_motor(0);
    }
    else
    {
        left_motor.run_motor(50);
        right_motor.run_motor(50);
    }
}

bool new_direction()
{
    int t = HAL_GetTick();
    unsigned int turns = rand() % 800;
    left_motor.run_motor(-55);
    right_motor.run_motor(35);
    while (HAL_GetTick() - t < turns)
    {
    }
    return true;
}

bool align()
{
    bool complete = false;
    int cm = sonar.ping_cm(); // take distance measurement
    int error = cm - TARGET_DISTANCE;
    // move forward
    if (error > 0)
    {
        right_motor.run_motor(20);
        left_motor.run_motor(20);
    }

    else if (error < 0)
    {
        float P = 1.4 * error;
        float D = 0;
        float adj = P + D;
        adj = (-1 * error * error) / 2.7;
        /* 
            The following depends on you. My left motor is always 
            slower than the right when going in reverse. So here I try 
            to apply various corrections 
        */
        int Lspeed = constrain(adj, -1 * 50, 0);
        int Rspeed = constrain(adj, -1 * 50, 0);

        // if (Lspeed > -1 * 40 && Lspeed < 0)
        // {
        //     Lspeed = map(Lspeed, -50, 0, -47, -30);
        // }
        right_motor.run_motor(Rspeed);
        left_motor.run_motor(Lspeed);
    }

    error = sonar.ping_cm() - TARGET_DISTANCE;

    if (abs(error) <= 3){
        complete = true;
        left_motor.run_motor(0);
        right_motor.run_motor(0);
    }
    return complete;
}

bool linear_search(short search_radius)
{
    int cm2 = sonar2.ping_cm(); // take distance measurement to see if it is a bin
    int cm = sonar.ping_cm();   // take distance measurement
    // if cm2 is 0, then that means object is outside of detection range
    if (cm2 == 0)
    {
        cm2 = 200;
    }
    if (cm < search_radius && cm != 0 && cm2 > cm * 2 && cm2 > 40)
    {
        return true;
    }
    else
    {
        run_both(50, 50, 120);
        run_both(0, 0, 80);
    }
    return false;
}

/**
  * Searches within the search radius for any objects
  * @param search_radius: centimeters
  * @param l_or_r: 1 is right turn search, -1 is left turn search
  */
bool search(int search_radius, int l_or_r)
{

    int cm2 = sonar2.ping_cm();         // take distance measurement to see if it is a bin
    int cm = sonar.ping_median(2) / 57; // take distance measurement

    // if cm2 is 0, then that means object is outside of detection range
    if (cm2 == 0)
    {
        cm2 = 200;
    }

    // object is within search radius
    // error will equal -1*TARGET_DISTANCE if the object is out of range
    // the top sensor detects at least twice the distance
    // cm2 has minimum 40 **THIS IS CAUSE MY SENSOR WILL NEVER RETURN VALUE LESS THAN 20**!!
    if (cm < search_radius && cm != 0 && cm2 > cm * 2 && cm2 > 50)
    {
        return true;
    }
    else
    {
        if (l_or_r == 1)
        {
            run_both(-55, 35, 50);
            run_both(0, 0, 140);
        }
        else
        {
            run_both(55, -35, 50);
            run_both(0, 0, 140);
        }
        return false;
    }
}