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

/** @file tree.c
 * @brief Tree functions.
 */

#include "mos.h"

#include "tree.h"
#include "mem.h"
#include "printf.h"
#include <stdbool.h>

void print_tree(node_ptr starting_point);
uint8_t right_rotate(node_ptr x);
uint8_t insert(uint16_t key);
node_ptr find_node(uint16_t key);
node_ptr get_max();
node_ptr get_min();
void print_tree_r(node_ptr starting_point,uint8_t depth);

node_ptr root = NULL;
uint8_t depth = 0;
uint16_t node_count=0;
uint16_t value_total;

node_ptr get_root()
{
   node_ptr ret_val;
   ret_val=root;
   return ret_val;
}

uint8_t right_rotate_node(uint16_t key)
{
   node_ptr x=find_node(key);
   if(x==NULL)
      return TREE_NODE_NOT_FOUND;
   return right_rotate(x);
}

uint8_t left_rotate_node(uint16_t key)
{
   node_ptr x=find_node(key);
   if(x==NULL)
      return TREE_NODE_NOT_FOUND;
   return left_rotate(x);
}

uint8_t right_rotate(node_ptr x)
{
   if(!x->left)
      return TREE_INVALID_ROTATION;

   x->left->p=x->p;                //adjust node's parent pointer
   
   if(x->p) {                        //we must adjust the parent
      if(x->p->right ==x)    
	 x->p->right=x->left;        //adjust parent's child (right) pointer
      else 
	 x->p->left=x->left;         //adjust parent's child (left) pointer
   }
   
   if(x->left->right)              //if we need to move subtree (beta)
      x->left->right->p=x;          //glue in subtree's parent pointer
   
   x->p=x->left;                   //change node's parent pointer
   x->left=x->left->right;         //glue in child pointer to subtree
   x->p->right=x;                  //change new parent's child ptr (to new node)
   
   return TREE_OK;
}

uint8_t left_rotate(node_ptr x)
{
   if(!x->right)
      return TREE_INVALID_ROTATION;

   x->right->p=x->p;              //adjust node's parent pointer
   
   if(x->p) {                      //we must adjust the parent
      if(x->p->left ==x)
	 x->p->left=x->right;       //adjust parent's child (left) pointer
      else //if(x->p->right ==x)
	 x->p->right=x->right;      //adjust parent's child (right) pointer
   }
   
   if(x->right->left)             //if we need to move subtree (beta)
    x->right->left->p=x;         //glue in subtree's parent pointer
   
   x->p=x->right;                 //change node's parent pointer
   x->right=x->right->left;       //glue in subtree child pointer to subtree
   x->p->left=x;                  //change new parent's child ptr (to new node)

   return TREE_OK;
}

node_ptr find_node(uint16_t key)
{
   node_ptr current=root;
   while(current){
      if(key == current->key)
	 return current;
      else if(key < current->key)
	 current=current->left;
      else
	 current=current->right;
   }
   return current;
}

void rtotal(node_ptr starting_point)
{
   if (starting_point==NULL) return;
   value_total += starting_point->key;
   rtotal(starting_point->left);
   rtotal(starting_point->right);
}

uint16_t get_total(node_ptr starting_point)
{
   value_total=0;
   rtotal(starting_point);
   return value_total;
}

void rcount_nodes(node_ptr starting_point)
{
   if(starting_point==NULL) return;
   node_count++;

   rcount_nodes(starting_point->left);
   rcount_nodes(starting_point->right);
}

uint16_t count_nodes(node_ptr starting_point)
{
   node_count=0;
   rcount_nodes(starting_point);
   return node_count;
}

node_ptr get_min()
{
   node_ptr current=root;
   if(current==NULL) return NULL;
   while(current->left)
      current=current->left;
   return current;
}

node_ptr get_max()
{
   node_ptr current=root;
   if(current==NULL) return NULL;
   while(current->right)
      current=current->right;
   return current;
}

uint8_t insert(uint16_t key)
{
   node_ptr current=root;
   node_ptr parent=NULL;
   node_ptr x;
   while(current) {
      parent=current;
      if(key == current->key){ //found a duplicate
	 return TREE_NODE_EXISTS;
      }
      else if(key < current->key){  //key must go in left subtree
	 current=current->left;
      }
      else{ //key must go in right subtree
	 current=current->right;
      }
      
   }
   x=(node_ptr)mos_mem_alloc(sizeof(struct node_t));
   if(x){
      x->p=parent;
      x->left=NULL;
      x->right=NULL;
      x->key=key;
   }
   else if(x==NULL){ //out of mem..
      printf("OUT OF MEMORY!!\n");
      return TREE_OUT_OF_MEM;
   }

   if(parent){
      if(key<parent->key)
	 parent->left=x;
      else if(key>parent->key)
	 parent->right=x;
   }
   else
      root=x;

   return TREE_OK;
}

void delete_subtree(node_ptr starting_point)
{
   /* delete right subtree */
   node_ptr cursor=starting_point;
   while(cursor){ //if the right tree exists...
      if(cursor->right)
	 cursor=cursor->right;

      else if(cursor->left)
	 cursor=cursor->left;

      else if(cursor->right==NULL &&
	      cursor->left==NULL){ //at a leaf
	 mos_mem_free((uint8_t *)cursor); //free the memory
	 if(cursor == root) break;
	 if(cursor == cursor->p->left) //left child
	    cursor->p->left=NULL;
	 if(cursor == cursor->p->right) //right child
	    cursor->p->right=NULL;
	 cursor=cursor->p; //walk back up the tree
      }//leaf
   }//while loop

   /* hopefully the tree is deleted and freed */
   /* from memory now, setting root to null. */

   root=NULL;
}

void clear_tree()
{
   delete_subtree(root);
}

void print_tabbed(uint8_t depth, uint16_t number)
{
   uint8_t i;
   for(i=0;i<depth;i++)
      printf("   ");
   printf("%d\n",number);
}

void show_tree()
{
   depth=0;
   print_tree(root);
}

void print_tree(node_ptr starting_point)
{
   depth=0;
   print_tree_r(root,0);
}

void print_tree_r(node_ptr starting_point, uint8_t depth)
{
  if (starting_point != NULL && depth < 13)
  {
     print_tree_r(starting_point->right,depth+1);
     print_tabbed(depth,starting_point->key);
     print_tree_r(starting_point->left,depth+1);
  }

}

uint8_t delete(uint16_t key)
{
   node_ptr x,y,z;
   
    /* start from the root */
    z = root;
    while(z != NULL) {
        if(key == z->key) 
            break;
        else if(key < z->key)
	   z=z->left;
	else
	   z=z->right;
    }
    if (!z) return TREE_NODE_NOT_FOUND;

    /* find the successor */
    if (z->left == NULL || z->right == NULL)
        y = z;
    else {
        y = z->right;
        while (y->left != NULL) y = y->left;
    }

    /* x is y's only child */
    if (y->left != NULL)
        x = y->left;
    else
        x = y->right;

    /* remove y from the parent chain */
    if (x) x->p = y->p;
    if (y->p)
        if (y == y->p->left)
            y->p->left = x;
        else
            y->p->right = x;
    else
        root = x;

    /* if z and y are not the same, replace z with y. */
    if (y != z) {
        y->left = z->left;
        if (y->left) y->left->p = y;
        y->right = z->right;
        if (y->right) y->right->p = y;
        y->p = z->p;
        if (z->p)
            if (z == z->p->left)
                z->p->left = y;
            else
                z->p->right = y;
        else
            root = y;
        mos_mem_free ((uint8_t *)z);
    } else {
       mos_mem_free ((uint8_t *)y);
    }
    return TREE_OK;
}

node_ptr find(uint16_t key)
{

    node_ptr current = root;
    while(current != NULL) {
        if(key == current->key)
	   return current;
	else if(key < current->key)
	   current=current->left;
	else
	   current=current->right;
        }
    return NULL;
}

void splay_node(uint16_t key)
{
   node_ptr x=find(key);
   if(x==NULL)
      printf("node not found.\n");
   else
      splay(x);
}

void splay(node_ptr x)
{
    while(x->p){                           //if node is root, we're done
      if(x->p->p==NULL){                   //zig case
	if(x->p->left == x)
	  right_rotate(x->p);
	else if(x->p->right == x)
	  left_rotate(x->p);
	break;
      }
      else if(x->p->left ==x){             //node is a left child
	if(x->p->p->left == x->p){         //zig-zig left ./
	  right_rotate(x->p->p);
	  right_rotate(x->p);
	}
	else {//if(x->p->p->right == x->p) //zig-zag left >
	  right_rotate(x->p);
	  left_rotate(x->p);
	}
      }
      else { //if(x->p->right ==x){        //node is a right child
	if(x->p->p->right == x->p){        //zig-zig right
	  left_rotate(x->p->p);
	  left_rotate(x->p);
	}
	else {// if(x->p->p->left == x->p) //zig-zag right <
	  left_rotate(x->p);
	  right_rotate(x->p);
	}
      } 
      
    }
  root=x;
}
