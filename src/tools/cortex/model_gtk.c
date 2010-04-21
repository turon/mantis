//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "cortex.h"
#include "gui_gtk.h"
#include "model_gtk.h"
#include "bionet-interface.h"

static GSList *add_node_funcs;
static GSList *remove_node_funcs;
static GSList *add_hab_funcs;
static GSList *remove_hab_funcs;
static GSList *update_resource_funcs;
static GSList *select_node_funcs;
static GSList *unselect_node_funcs;
static GSList *select_hab_funcs;
static GSList *unselect_hab_funcs;

void model_add_node_register_func(model_add_node_f func)
{
   add_node_funcs = g_slist_append(add_node_funcs, func);
}

void model_add_node(bionet_node_t *node)
{
   if(node == NULL)
      return;
   
   GSList *cur;

   for(cur = add_node_funcs; cur != NULL; cur = cur->next) {
      model_add_node_f func = cur->data;
      func(node);
   }
}

void model_remove_node_register_func(model_remove_node_f func)
{
   remove_node_funcs = g_slist_append(remove_node_funcs, func);   
}

void model_remove_node(bionet_node_t *node)
{
   if(node == NULL)
      return;
   
   GSList *cur;

   for(cur = remove_node_funcs; cur != NULL; cur = cur->next) {
      model_remove_node_f func = cur->data;
      func(node);
   }   
}

void model_add_hab_register_func(model_add_hab_f func)
{
   add_hab_funcs = g_slist_append(add_hab_funcs, func);
}

void model_add_hab(bionet_hab_t *hab)
{
   debug("GOT add hab");
   
   if(hab == NULL)
      return;
   
   GSList *cur;

   for(cur = add_hab_funcs; cur != NULL; cur = cur->next) {
      model_add_hab_f func = cur->data;
      func(hab);
   }   
}

void model_remove_hab_register_func(model_remove_hab_f func)
{
   remove_hab_funcs = g_slist_append(remove_hab_funcs, func);
}

void model_remove_hab(bionet_hab_t *hab)
{

   if(hab == NULL)
      return;
   
   GSList *cur;

   for(cur = remove_hab_funcs; cur != NULL; cur = cur->next) {
      model_remove_hab_f func = cur->data;
      func(hab);
   }
}

void model_update_resource_register_func(model_update_resource_f func)
{
   update_resource_funcs = g_slist_append(update_resource_funcs, func);
}

void model_update_resource(bionet_resource_t *resource)
{
   if(resource == NULL)
      return;
   
   GSList *cur;

   for(cur = update_resource_funcs; cur != NULL; cur = cur->next) {
      model_update_resource_f func = cur->data;
      func(resource);
   }
}

void model_select_node_register_func(model_select_node_f func)
{
   select_node_funcs = g_slist_append(select_node_funcs, func);
}

void model_select_node_unregister_func(model_select_node_f func)
{
   select_node_funcs = g_slist_remove(select_node_funcs, func);
}


void model_select_node(bionet_node_t *node)
{
   GSList *cur;

   for(cur = select_node_funcs; cur != NULL; cur = cur->next) {
      model_select_node_f func = cur->data;
      func(node);
   }
}

void model_select_node_skip(bionet_node_t *node, model_select_node_f skip_func)
{
   if(node == NULL)
      return;
   
   GSList *cur;
   
   for(cur = select_node_funcs; cur != NULL; cur = cur->next) {
      debug("calling a select node func");
      model_select_node_f func = cur->data;
      if(func != skip_func)
	 func(node);
   }
}


void model_unselect_node_register_func(model_unselect_node_f func)
{
   unselect_node_funcs = g_slist_append(unselect_node_funcs, func);
}

void model_unselect_node(bionet_node_t *node)
{
   GSList *cur;

   for(cur = unselect_node_funcs; cur != NULL; cur = cur->next) {
      model_unselect_node_f func = cur->data;
      func(node);
   }
}

void model_select_hab_register_func(model_select_hab_f func)
{
   select_hab_funcs = g_slist_append(select_hab_funcs, func);
}

void model_select_hab_unregister_func(model_select_hab_f func)
{
   select_hab_funcs = g_slist_remove(select_hab_funcs, func);
}

void model_select_hab(bionet_hab_t *hab)
{

   if(hab == NULL)
      return;
   
   GSList *cur;

   for(cur = select_hab_funcs; cur != NULL; cur = cur->next) {
      debug("calling a select hab func");
      model_select_hab_f func = cur->data;
      func(hab);
   }
}

void model_select_hab_skip(bionet_hab_t *hab, model_select_hab_f skip_func)
{
   if(hab == NULL)
      return;
   
   GSList *cur;
   
   for(cur = select_hab_funcs; cur != NULL; cur = cur->next) {
      debug("calling a select hab func");
      model_select_hab_f func = cur->data;
      if(func != skip_func)
	 func(hab);
   }
}

void model_unselect_hab_unregister_func(model_unselect_hab_f func)
{
   unselect_hab_funcs = g_slist_append(unselect_hab_funcs, func);
}

void model_unselect_hab(bionet_hab_t *hab)
{

   if(hab == NULL)
      return;
   
   GSList *cur;

   for(cur = unselect_hab_funcs; cur != NULL; cur = cur->next) {
      model_unselect_hab_f func = cur->data;
      func(hab);
   }
}

