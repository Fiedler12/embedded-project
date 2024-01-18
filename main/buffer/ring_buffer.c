#include "ring_buffer.h"

struct Ring_buffer* init_buffer(int size) {
    // First, allocate our samples
    struct Sample *samples = (struct Sample*) malloc(sizeof(struct Sample) * size);
    struct Buffer_element *elements = (struct Buffer_element*) malloc(sizeof(struct Buffer_element) * size);
    struct Ring_buffer *buffer = (struct Ring_buffer*) malloc(sizeof(struct Ring_buffer));

    buffer->size = size;
    buffer->head = &elements[0];
    buffer->tail = &elements[0];

    for(int i = 0; i < size; i++) {
        elements[i].values = &samples[i];
        samples[i].light_level = i;
        if (i < size -1) {
            elements[i].next = &elements[i+1];
        } else {
            elements[i].next = buffer->head;
        }
    }
    printf("init: Address of head is: %p\n", buffer->head);
    return buffer;
}

void print_buffer(struct Ring_buffer *buffer) {
    struct Buffer_element *element = buffer->head;
    for (int i = 0; i < buffer->size; i++)
    {
        struct Sample *sample = element->values;
        printf("Sample is: %f, %d, %u\n", sample->temperature, sample->light_level, sample->moisture);
        element = element->next;
    }    
}

void input_sample(struct Ring_buffer *buffer, struct Sample *sample) {
    struct Sample *tail_sample = buffer->tail->values;
    tail_sample->light_level = sample->light_level;
    tail_sample->moisture = sample->moisture; 
    tail_sample->temperature = sample->temperature;
    buffer->tail = buffer->tail->next;
    if (buffer->tail == buffer->head) {
        buffer->head = buffer->head->next;
    }
}
