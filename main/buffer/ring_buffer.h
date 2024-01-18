#include <stdio.h>
#include <stdlib.h>

struct Sample {
  float temperature;
  int light_level;
  unsigned short moisture; 
};

struct Buffer_element {
    struct Sample *values;
    struct Buffer_element *next;
};

struct Ring_buffer {
    struct Buffer_element *head; 
    struct Buffer_element *tail;
    int size; 
};

struct Ring_buffer* init_buffer(int size);

void print_buffer(struct Ring_buffer *buffer);

void input_sample(struct Ring_buffer *buffer, struct Sample *sample);
