#ifndef SEARCH_H
#define SEARCH_H

#include <main.h>
#include <Hardware.h>

#define MAX_DISTANCE 60
#define TARGET_DISTANCE 30

struct Search
{
    Search(int max_search_fail, int max_align_fail, int max_linear_search_fail) : max_search_fail(max_search_fail), max_align_fail(max_align_fail), max_linear_search_fail(max_linear_search_fail)
    {
        linear_search_counter = max_linear_search_fail;
    };
    void loop();
    bool enter_arena();

private:
    int max_search_fail = 0;
    int max_align_fail = 0;
    int search_fail = 0;
    int align_fail = 0;
    int state = 1;
    int max_linear_search_fail = 0;
    int linear_search_counter = 0;
    int direction = 1; // 1 is right -1 is left

    void update_state(bool result);
};

bool align();
bool search(int search_radius, int l_or_r);
bool linear_search(short search_radius);
bool new_direction(short direction);

#endif