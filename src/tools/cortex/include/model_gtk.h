//  This file is part of MOS, the MANTIS Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (c) 2002 - 2007 University of Colorado, Boulder
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//       * Redistributions of source code must retain the above copyright
//         notice, this list of conditions and the following disclaimer.
//       * Redistributions in binary form must reproduce the above
//         copyright notice, this list of conditions and the following
//         disclaimer in the documentation and/or other materials provided
//         with the distribution. 
//       * Neither the name of the MANTIS Project nor the names of its
//         contributors may be used to endorse or promote products derived
//         from this software without specific prior written permission.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.

#include "bionet.h"

typedef void (*model_add_node_f)(bionet_node_t *node);
typedef void (*model_remove_node_f)(bionet_node_t *node);
typedef void (*model_add_hab_f)(bionet_hab_t *hab);
typedef void (*model_remove_hab_f)(bionet_hab_t *hab);
typedef void (*model_update_resource_f)(bionet_resource_t *resource);
typedef void (*model_select_node_f)();
typedef void (*model_unselect_node_f)();
typedef void (*model_select_hab_f)(bionet_hab_t *hab);
typedef void (*model_unselect_hab_f)();


void model_add_node_register_func(model_add_node_f func);
void model_add_node(bionet_node_t *node);
void model_remove_node_register_func(model_remove_node_f func);
void model_remove_node(bionet_node_t *node);

void model_add_hab_register_func(model_add_hab_f func);
void model_add_hab(bionet_hab_t *hab);
void model_remove_hab_register_func(model_remove_hab_f func);
void model_remove_hab(bionet_hab_t *hab);

void model_update_resource_register_func(model_update_resource_f func);
void model_update_resource(bionet_resource_t *resource);

void model_select_node_register_func(model_select_node_f func);
void model_select_node_unregister_func(model_select_node_f func);
void model_select_node(bionet_node_t *node);
void model_select_node_skip(bionet_node_t *node, model_select_node_f skip_func);
void model_unselect_node_register_func(model_unselect_node_f func);
void model_unselect_node(bionet_node_t *node);

void model_select_hab_register_func(model_select_hab_f func);
void model_select_hab_unregister_func(model_select_hab_f func);
void model_select_hab(bionet_hab_t *hab);
void model_select_hab_skip(bionet_hab_t *hab, model_select_hab_f skip_func);
void model_unselect_hab_unregister_func(model_unselect_hab_f func);
void model_unselect_hab(bionet_hab_t *hab);

