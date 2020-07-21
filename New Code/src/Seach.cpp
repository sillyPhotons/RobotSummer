#include <Search.h>

void Search::loop()
{
    switch (state)
    {
    case 1: // spherical search
        update_state(search(MAX_DISTANCE, 1));
        break;
    case 2: // alignment
        update_state(align());
        break;
    case 3: // pickup cans
        pick_up_can(true);
        delay(1000);
        update_state(true);
        break;
    case 4: // new direction
        update_state(new_direction());
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
            state = 4;
        }
        else
        {
            linear_search_counter = max_linear_search_fail;
            state = 1;
        }
        break;
    }
}

bool new_direction()
{
    int t = HAL_GetTick();
    unsigned int turns = rand() % 800;
    while (HAL_GetTick() - t < turns)
    {
        left_motor.run_motor(-55);
        right_motor.run_motor(35);
    }
    return linear_search(MAX_DISTANCE);
}

bool align()
{
    int cm = sonar.ping_cm(); // take distance measurement
    int error = cm - TARGET_DISTANCE;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("ERROR: ");
    display.println(error);

    // move forward
    if (error > 0)
    {
        left_motor.run_motor(20);
        right_motor.run_motor(20);
    }

    else if (error < 0)
    {
        float P = 1.2 * error;
        float D = 0;
        float adj = P + D;

        /* 
            The following depends on you. My left motor is always 
            slower than the right when going in reverse. So here I try 
            to apply various corrections 
        */
        int Lspeed = constrain(adj, -1 * 50, 40);
        int Rspeed = constrain(adj, -1 * 50, 40);

        if (Lspeed > -1 * 40 && Lspeed < 0)
        {
            Lspeed = map(Lspeed, -50, 0, -47, -30);
        }
        left_motor.run_motor(Lspeed);
        right_motor.run_motor(Rspeed);
        display.println(Lspeed);
        display.println(Rspeed);
        display.display();
    }

    error = sonar.ping_cm() - TARGET_DISTANCE;
    bool complete = false;
    if (abs((int)error) < 3)
    {
        complete = true;
    }
    return complete;
}

bool linear_search(short search_radius)
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("LINE SEARCH");
    display.display();

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
        run2_for_ms(&left_motor, &right_motor, 40, 40, 100);
        run2_for_ms(&left_motor, &right_motor, 0, 0, 100);
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

    int cm2 = sonar2.ping_cm(); // take distance measurement to see if it is a bin
    int cm = sonar.ping_cm();   // take distance measurement

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
            run2_for_ms(&left_motor, &right_motor, -55, 35, 50);
            run2_for_ms(&left_motor, &right_motor, 0, 0, 150);
        }
        else
        {
            run2_for_ms(&left_motor, &right_motor, 35, -35, 50);
            run2_for_ms(&left_motor, &right_motor, 0, 0, 150);
        }
        return false;
    }
}