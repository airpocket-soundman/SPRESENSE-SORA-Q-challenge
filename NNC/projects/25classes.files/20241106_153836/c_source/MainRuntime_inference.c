// Copyright (c) 2017 Sony Corporation. All Rights Reserved.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// *WARNING*
// THIS FILE IS AUTO-GENERATED BY CODE GENERATOR.
// PLEASE DO NOT EDIT THIS FILE BY HAND!

#include "MainRuntime_inference.h"

#include <nnablart/functions.h>
#include <string.h>

typedef struct {
    void *buffer_pool[2];
    void *param_pool[4];
    rt_buffer_allocate_type_t variable_buffers_allocate_type[10];

    // Variables
    rt_variable_t v0; ///< Input
    int v0_shape[4];
    rt_variable_t v1; ///< Convolution/conv/W
    int v1_shape[4];
    rt_variable_t v2; ///< Convolution/conv/b
    int v2_shape[1];
    rt_variable_t v3; ///< Affine/affine/W
    int v3_shape[2];
    rt_variable_t v4; ///< Affine/affine/b
    int v4_shape[1];
    rt_variable_t v5; ///< Convolution
    int v5_shape[4];
    rt_variable_t v6; ///< MaxPooling
    int v6_shape[4];
    rt_variable_t v7; ///< ReLU
    int v7_shape[4];
    rt_variable_t v8; ///< Affine
    int v8_shape[2];
    rt_variable_t v9; ///< Softmax
    int v9_shape[2];

    // Functions
    rt_function_t f0; ///< Convolution
    rt_variable_t* f0_inputs[3];
    rt_variable_t* f0_outputs[1];
    convolution_local_context_t *f0_local_context;
    int f0_local_context_shape_pad[2];
    int f0_local_context_shape_stride[2];
    int f0_local_context_shape_dilation[2];
    rt_function_t f1; ///< MaxPooling
    rt_variable_t* f1_inputs[1];
    rt_variable_t* f1_outputs[1];
    max_pooling_local_context_t *f1_local_context;
    int f1_local_context_shape_kernel[2];
    int f1_local_context_shape_stride[2];
    int f1_local_context_shape_pad[2];
    rt_function_t f2; ///< ReLU
    rt_variable_t* f2_inputs[1];
    rt_variable_t* f2_outputs[1];
    relu_local_context_t *f2_local_context;
    rt_function_t f3; ///< Affine
    rt_variable_t* f3_inputs[3];
    rt_variable_t* f3_outputs[1];
    affine_local_context_t *f3_local_context;
    rt_function_t f4; ///< Softmax
    rt_variable_t* f4_inputs[1];
    rt_variable_t* f4_outputs[1];
    softmax_local_context_t *f4_local_context;
} nnablart_mainruntime_local_context_t;

int actual_buf_sizes[2] = {
    784,
    10000,
};

void *(*rt_variable_malloc_func)(size_t size) = malloc;
void (*rt_variable_free_func)(void *ptr) = free;

void *(*rt_malloc_func)(size_t size) = malloc;
void (*rt_free_func)(void *ptr) = free;


void* nnablart_mainruntime_allocate_context(void** params)
{
    nnablart_mainruntime_local_context_t* c = malloc(sizeof(nnablart_mainruntime_local_context_t));
    // Variable buffer
    for (int i = 0; i < 2; i++) {
        c->buffer_pool[i] = malloc(sizeof(float) * actual_buf_sizes[i]);
        memset(c->buffer_pool[i], 0, sizeof(float) * actual_buf_sizes[i]);
    }
    if(params) {
        c->variable_buffers_allocate_type[1] = RT_BUFFER_ALLOCATE_TYPE_ALLOCATED;
        c->param_pool[0] = *params++;
        c->variable_buffers_allocate_type[2] = RT_BUFFER_ALLOCATE_TYPE_ALLOCATED;
        c->param_pool[1] = *params++;
        c->variable_buffers_allocate_type[3] = RT_BUFFER_ALLOCATE_TYPE_ALLOCATED;
        c->param_pool[2] = *params++;
        c->variable_buffers_allocate_type[4] = RT_BUFFER_ALLOCATE_TYPE_ALLOCATED;
        c->param_pool[3] = *params++;
    } else {
        c->variable_buffers_allocate_type[1] = RT_BUFFER_ALLOCATE_TYPE_MALLOC;
        c->param_pool[0] = *params++;
        c->param_pool[0] = malloc(sizeof(float) * 256);
        c->variable_buffers_allocate_type[2] = RT_BUFFER_ALLOCATE_TYPE_MALLOC;
        c->param_pool[1] = *params++;
        c->param_pool[1] = malloc(sizeof(float) * 16);
        c->variable_buffers_allocate_type[3] = RT_BUFFER_ALLOCATE_TYPE_MALLOC;
        c->param_pool[2] = *params++;
        c->param_pool[2] = malloc(sizeof(float) * 14400);
        c->variable_buffers_allocate_type[4] = RT_BUFFER_ALLOCATE_TYPE_MALLOC;
        c->param_pool[3] = *params++;
        c->param_pool[3] = malloc(sizeof(float) * 25);
    }

    // Variables
    // Input
    (c->v0).type = NN_DATA_TYPE_FLOAT;
    (c->v0).shape.size = 4;
    c->v0_shape[0] = 1;
    c->v0_shape[1] = 1;
    c->v0_shape[2] = 28;
    c->v0_shape[3] = 28;
    (c->v0).shape.data = c->v0_shape;
    (c->v0).data = c->buffer_pool[0];
    // Convolution/conv/W
    (c->v1).type = NN_DATA_TYPE_FLOAT;
    (c->v1).shape.size = 4;
    c->v1_shape[0] = 16;
    c->v1_shape[1] = 1;
    c->v1_shape[2] = 4;
    c->v1_shape[3] = 4;
    (c->v1).shape.data = c->v1_shape;
    (c->v1).data = c->param_pool[0];
    // Convolution/conv/b
    (c->v2).type = NN_DATA_TYPE_FLOAT;
    (c->v2).shape.size = 1;
    c->v2_shape[0] = 16;
    (c->v2).shape.data = c->v2_shape;
    (c->v2).data = c->param_pool[1];
    // Affine/affine/W
    (c->v3).type = NN_DATA_TYPE_FLOAT;
    (c->v3).shape.size = 2;
    c->v3_shape[0] = 576;
    c->v3_shape[1] = 25;
    (c->v3).shape.data = c->v3_shape;
    (c->v3).data = c->param_pool[2];
    // Affine/affine/b
    (c->v4).type = NN_DATA_TYPE_FLOAT;
    (c->v4).shape.size = 1;
    c->v4_shape[0] = 25;
    (c->v4).shape.data = c->v4_shape;
    (c->v4).data = c->param_pool[3];
    // Convolution
    (c->v5).type = NN_DATA_TYPE_FLOAT;
    (c->v5).shape.size = 4;
    c->v5_shape[0] = 1;
    c->v5_shape[1] = 16;
    c->v5_shape[2] = 25;
    c->v5_shape[3] = 25;
    (c->v5).shape.data = c->v5_shape;
    (c->v5).data = c->buffer_pool[1];
    // MaxPooling
    (c->v6).type = NN_DATA_TYPE_FLOAT;
    (c->v6).shape.size = 4;
    c->v6_shape[0] = 1;
    c->v6_shape[1] = 16;
    c->v6_shape[2] = 6;
    c->v6_shape[3] = 6;
    (c->v6).shape.data = c->v6_shape;
    (c->v6).data = c->buffer_pool[0];
    // ReLU
    (c->v7).type = NN_DATA_TYPE_FLOAT;
    (c->v7).shape.size = 4;
    c->v7_shape[0] = 1;
    c->v7_shape[1] = 16;
    c->v7_shape[2] = 6;
    c->v7_shape[3] = 6;
    (c->v7).shape.data = c->v7_shape;
    (c->v7).data = c->buffer_pool[1];
    // Affine
    (c->v8).type = NN_DATA_TYPE_FLOAT;
    (c->v8).shape.size = 2;
    c->v8_shape[0] = 1;
    c->v8_shape[1] = 25;
    (c->v8).shape.data = c->v8_shape;
    (c->v8).data = c->buffer_pool[0];
    // Softmax
    (c->v9).type = NN_DATA_TYPE_FLOAT;
    (c->v9).shape.size = 2;
    c->v9_shape[0] = 1;
    c->v9_shape[1] = 25;
    (c->v9).shape.data = c->v9_shape;
    (c->v9).data = c->buffer_pool[1];

    // Functions
    // Convolution
    c->f0_local_context = malloc(sizeof(convolution_local_context_t));
    (c->f0).num_of_inputs = 3;
    (c->f0_inputs)[0] = &(c->v0);
    (c->f0_inputs)[1] = &(c->v1);
    (c->f0_inputs)[2] = &(c->v2);
    (c->f0).inputs = c->f0_inputs;
    (c->f0).num_of_outputs = 1;
    (c->f0_outputs)[0] = &(c->v5);
    (c->f0).outputs = c->f0_outputs;
    (c->f0).local_context = c->f0_local_context;
    (c->f0_local_context)->base_axis = 1;
    rt_list_t arg_f0_pad;
    arg_f0_pad.size = 2;
    arg_f0_pad.data = c->f0_local_context_shape_pad;
    arg_f0_pad.data[0] = 0;
    arg_f0_pad.data[1] = 0;
    (c->f0_local_context)->pad = arg_f0_pad;
    rt_list_t arg_f0_stride;
    arg_f0_stride.size = 2;
    arg_f0_stride.data = c->f0_local_context_shape_stride;
    arg_f0_stride.data[0] = 1;
    arg_f0_stride.data[1] = 1;
    (c->f0_local_context)->stride = arg_f0_stride;
    rt_list_t arg_f0_dilation;
    arg_f0_dilation.size = 2;
    arg_f0_dilation.data = c->f0_local_context_shape_dilation;
    arg_f0_dilation.data[0] = 1;
    arg_f0_dilation.data[1] = 1;
    (c->f0_local_context)->dilation = arg_f0_dilation;
    (c->f0_local_context)->group = 1;
    (c->f0_local_context)->channel_last = 0;
    allocate_convolution_local_context(&(c->f0));
    // MaxPooling
    c->f1_local_context = malloc(sizeof(max_pooling_local_context_t));
    (c->f1).num_of_inputs = 1;
    (c->f1_inputs)[0] = &(c->v5);
    (c->f1).inputs = c->f1_inputs;
    (c->f1).num_of_outputs = 1;
    (c->f1_outputs)[0] = &(c->v6);
    (c->f1).outputs = c->f1_outputs;
    (c->f1).local_context = c->f1_local_context;
    rt_list_t arg_f1_kernel;
    arg_f1_kernel.size = 2;
    arg_f1_kernel.data = c->f1_local_context_shape_kernel;
    arg_f1_kernel.data[0] = 4;
    arg_f1_kernel.data[1] = 4;
    (c->f1_local_context)->kernel = arg_f1_kernel;
    rt_list_t arg_f1_stride;
    arg_f1_stride.size = 2;
    arg_f1_stride.data = c->f1_local_context_shape_stride;
    arg_f1_stride.data[0] = 4;
    arg_f1_stride.data[1] = 4;
    (c->f1_local_context)->stride = arg_f1_stride;
    (c->f1_local_context)->ignore_border = 1;
    rt_list_t arg_f1_pad;
    arg_f1_pad.size = 2;
    arg_f1_pad.data = c->f1_local_context_shape_pad;
    arg_f1_pad.data[0] = 0;
    arg_f1_pad.data[1] = 0;
    (c->f1_local_context)->pad = arg_f1_pad;
    (c->f1_local_context)->channel_last = 0;
    allocate_max_pooling_local_context(&(c->f1));
    // ReLU
    c->f2_local_context = malloc(sizeof(relu_local_context_t));
    (c->f2).num_of_inputs = 1;
    (c->f2_inputs)[0] = &(c->v6);
    (c->f2).inputs = c->f2_inputs;
    (c->f2).num_of_outputs = 1;
    (c->f2_outputs)[0] = &(c->v7);
    (c->f2).outputs = c->f2_outputs;
    (c->f2).local_context = c->f2_local_context;
    (c->f2_local_context)->inplace = 1;
    allocate_relu_local_context(&(c->f2));
    // Affine
    c->f3_local_context = malloc(sizeof(affine_local_context_t));
    (c->f3).num_of_inputs = 3;
    (c->f3_inputs)[0] = &(c->v7);
    (c->f3_inputs)[1] = &(c->v3);
    (c->f3_inputs)[2] = &(c->v4);
    (c->f3).inputs = c->f3_inputs;
    (c->f3).num_of_outputs = 1;
    (c->f3_outputs)[0] = &(c->v8);
    (c->f3).outputs = c->f3_outputs;
    (c->f3).local_context = c->f3_local_context;
    (c->f3_local_context)->base_axis = 1;
    allocate_affine_local_context(&(c->f3));
    // Softmax
    c->f4_local_context = malloc(sizeof(softmax_local_context_t));
    (c->f4).num_of_inputs = 1;
    (c->f4_inputs)[0] = &(c->v8);
    (c->f4).inputs = c->f4_inputs;
    (c->f4).num_of_outputs = 1;
    (c->f4_outputs)[0] = &(c->v9);
    (c->f4).outputs = c->f4_outputs;
    (c->f4).local_context = c->f4_local_context;
    (c->f4_local_context)->axis = 1;
    allocate_softmax_local_context(&(c->f4));
    return (void*)c;
}

int nnablart_mainruntime_free_context(void* context)
{
    nnablart_mainruntime_local_context_t* c = (nnablart_mainruntime_local_context_t*)context;

    for (int i = 0; i < 2; i++) {
        free(c->buffer_pool[i]);
    }
    if(c->variable_buffers_allocate_type[1] == RT_BUFFER_ALLOCATE_TYPE_MALLOC) {
        free(c->param_pool[0]);
    }
    if(c->variable_buffers_allocate_type[2] == RT_BUFFER_ALLOCATE_TYPE_MALLOC) {
        free(c->param_pool[1]);
    }
    if(c->variable_buffers_allocate_type[3] == RT_BUFFER_ALLOCATE_TYPE_MALLOC) {
        free(c->param_pool[2]);
    }
    if(c->variable_buffers_allocate_type[4] == RT_BUFFER_ALLOCATE_TYPE_MALLOC) {
        free(c->param_pool[3]);
    }
    free_convolution_local_context(&(c->f0));
    if (c->f0.local_context != 0) {
        rt_free_func(c->f0.local_context);
        c->f0.local_context = 0;
    }
    free_max_pooling_local_context(&(c->f1));
    if (c->f1.local_context != 0) {
        rt_free_func(c->f1.local_context);
        c->f1.local_context = 0;
    }
    free_relu_local_context(&(c->f2));
    if (c->f2.local_context != 0) {
        rt_free_func(c->f2.local_context);
        c->f2.local_context = 0;
    }
    free_affine_local_context(&(c->f3));
    if (c->f3.local_context != 0) {
        rt_free_func(c->f3.local_context);
        c->f3.local_context = 0;
    }
    free_softmax_local_context(&(c->f4));
    if (c->f4.local_context != 0) {
        rt_free_func(c->f4.local_context);
        c->f4.local_context = 0;
    }  
    free(context);
    return NN_ERROR_CODE_NOERROR;
}

float* nnablart_mainruntime_input_buffer(void* context, int index)
{
    nnablart_mainruntime_local_context_t* c = (nnablart_mainruntime_local_context_t*)context;
    switch(index) {
        case 0: return (c->v0).data;
    }
    return 0;
}

float* nnablart_mainruntime_output_buffer(void* context, int index)
{
    nnablart_mainruntime_local_context_t* c = (nnablart_mainruntime_local_context_t*)context;
    switch(index) {
        case 0: return (c->v9).data;
    }

    return 0;
}
float* nnablart_mainruntime_param_buffer(void* context, int index)
{
    nnablart_mainruntime_local_context_t* c = (nnablart_mainruntime_local_context_t*)context;
    switch(index) {
        case 0: return (c->v3).data;
        case 1: return (c->v4).data;
        case 2: return (c->v1).data;
        case 3: return (c->v2).data;
    }
    return 0;
}
int nnablart_mainruntime_inference(void* context)
{
    nnablart_mainruntime_local_context_t* c = (nnablart_mainruntime_local_context_t*)context;
    exec_convolution(&(c->f0));
    exec_max_pooling(&(c->f1));
    exec_relu(&(c->f2));
    exec_affine(&(c->f3));
    exec_softmax(&(c->f4));
    return NN_ERROR_CODE_NOERROR;
}
