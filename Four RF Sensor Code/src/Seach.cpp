#include <Search.h>

void Search::loop()
{
    switch (state)
    {
    case 1: // spherical search
        update_state(search(MAX_DISTANCE, 1));
        break;
    case 2: // alignment
    {
        bool result = align();
        update_state(result);
        break;
    }
    case 3: // pickup cans
        pick_up_can(true);
        delay(200);
        update_state(true);
        break;
    case 4: // new direction
        update_state(new_direction(direction));
        break;
    case 5: // linear search
        update_state(linear_search(MAX_DISTANCE));
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
        }
        if (search_fail > max_search_fail)
        {
            search_fail = 0;
            // direction *= -1;
            // direction *= 1;
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
        direction *= -1;
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

bool Search::enter_arena()
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
        return true;
    }
    else
    {
        left_motor.run_motor(50);
        right_motor.run_motor(50);
    }
    return false;
}
/**
 * @param direction: 1: turn right, -1 turn left
 * */
bool new_direction(short direction)
{
    int t = HAL_GetTick();
    unsigned int turns = rand() % 880;
    if (direction == 1)
    {
        left_motor.run_motor(-55);
        right_motor.run_motor(35);
    }
    else
    {
        left_motor.run_motor(35);
        right_motor.run_motor(-55);
    }
    while (HAL_GetTick() - t < turns)
    {
    }
    return true;
}

bool align()
{
    bool complete = false;
    int cm = sonar.ping_median(2) / 57; // take distance measurement
    int error = cm - TARGET_DISTANCE;
    // Serial1.printf("\tResult: %d\n", error);
    // move forward
    if (error > 0)
    {
        // Serial1.printf("\t\tForward\n");
        left_motor.run_motor(30);
        right_motor.run_motor(30);
    }

    else if (error < 0)
    {
        float P = 1.4 * error;
        float D = 0;
        float adj = P + D;
        adj = (-1 * error * error) / 2.7;
        int Lspeed = constrain(adj, -40, -10); // changed
        int Rspeed = constrain(adj, -40, -10);
        left_motor.run_motor(Lspeed);
        right_motor.run_motor(Rspeed);
    }

    error = sonar.ping_median(2) / 57 - TARGET_DISTANCE;
    // Serial1.printf("\tResult: %d\n", error);
    if (abs(error) <= 3)
    {
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
        bool mot = run_both(50, 50, 120, true);
        if (!mot)
        {
            return false;
        }
        run_both(0, 0, 80, true);
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
            run_both(-55, 35, 55, true);
        }
        else
        {
            run_both(55, -35, 55, true);
        }
        run_both(0, 0, 50, true);
        return false;
    }
}