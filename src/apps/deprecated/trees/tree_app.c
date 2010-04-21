//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/* file: trees_app.c
   author: Charles Gruenwald III
   description: this application provides an interface to the simple
   tree library.
*/


/* warning this will fragment the memory if other threads are running */

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "command_daemon.h"
#include "printf.h"
#include "com.h"
#include "led.h"
#include "dev.h"
#include "tree.h"

void add_num(uint16_t new_node_key);

void add_node(){
   uint16_t new_node_key;
   new_node_key = prompt_long("new node's key:");
   add_num(new_node_key);
   
}
void del_node(){
   uint16_t key_to_delete;
   uint8_t result;
   key_to_delete=prompt_long("key to remove: ");
   result = delete(key_to_delete);
   if(result == TREE_NODE_NOT_FOUND){
      printf("node [%d] not found.\n",key_to_delete);
   }
   else if(result == TREE_OK){
      printf("node [%d] properly deleted.\n",key_to_delete);
   }
}
void printtree(){
   print_tree(get_root());
}
void add_num(uint16_t new_node_key){
   uint8_t result;
   result = insert(new_node_key);
   if(result == TREE_OUT_OF_MEM)
   {
      printf("no more space to add nodes.\n");
   }
   else if(result== TREE_OK)
      printf("node %d properly added.\n",new_node_key);
}

void fill_seq(){
   uint16_t min;
   uint16_t max;
   uint16_t counter;
   uint8_t keep_asking=true;
   uint8_t result;
   if(keep_asking){
      min=prompt_long("minimum value to start from:");
      max=prompt_long("maximum value to insert to:");
      if(min <= max) keep_asking=false;
   }

   for(counter=min;counter<=max;counter++)
   {
      result = insert(counter);
      if(result == TREE_OUT_OF_MEM)      {
	 printf("Not enough memory.\n");
	 break;
      }
      else if(result== TREE_OK)
	 printf("node %d properly added.\n",counter);
   }
}
void splay_node_local(){
   uint16_t node_key;
   node_key = prompt_long("which node do you want to splay:");
   splay_node(node_key);
}
void find_node_local(){
   uint16_t node_key;
   node_ptr search_result;
   node_key = prompt_long("which node do you wish to find:");
   search_result = (node_ptr)find_node(node_key);
   if(search_result == NULL)
      printf("Node [ %l ] not found.\n",node_key);
   else
      printf("Node [ %l ] was found.\n",node_key);
      
}
void clear_tree_local(){
   clear_tree();
}
void get_min_local(){
   node_ptr search;
   search= (node_ptr)get_min();
   if(search==NULL)
      printf("empty tree.\n");
   else
      printf("minimum value: %l\n",search->key);
}
void get_max_local(){
   node_ptr search;
   search = (node_ptr)get_max();
   if(search==(node_ptr)NULL)
      printf("empty tree.\n");
   else
      printf("maximum value: %l\n",search->key);
}
void get_total_local(){
   uint16_t shorter;
   shorter=get_total(get_root());
   printf("total for all keys is : %d\n",shorter);
}
void count_nodes_local(){
   uint16_t local_node_count=0;
   node_ptr start;
   start=get_root();
   local_node_count=count_nodes(start);
   printf("number of keys: %d\n",local_node_count);
}
void defaults(){
   uint16_t counter;
   add_num(50);
   for(counter =5;counter < 100;counter+=2){
      
      add_num(counter);
   }
   splay_node(25);
   splay_node(71);
   splay_node(31);
}
void get_avg_local(){
   uint16_t total;
   uint16_t count;
   uint16_t avg;
   total=get_total(get_root());
   count=count_nodes(get_root());
   avg=total/count;
   printf("the average is : %d\n",avg);
}
void start(void)
{
   mos_thread_new(mos_command_daemon,512,PRIORITY_NORMAL);
   mos_register_function("defaults", (void *)defaults);
   mos_register_function("add",      (void *)add_node);
   mos_register_function("delete",   (void *)del_node);
   mos_register_function("print",    (void *)printtree);
   mos_register_function("splay",    (void *)splay_node_local);
   mos_register_function("min",      (void *)get_min_local);
   mos_register_function("max",      (void *)get_max_local);
   mos_register_function("find",     (void *)find_node_local);
   mos_register_function("clear",    (void *)clear_tree_local);
   mos_register_function("count",    (void *)count_nodes_local);
   mos_register_function("total",    (void *)get_total_local);
   mos_register_function("average",  (void *)get_avg_local);
}

