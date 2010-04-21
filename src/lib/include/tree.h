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

/** @file tree.h
 * @brief A small tree library for the sensor node.
 *
 * Note: These routines may blow the stack or cause fragmentation
 * in memory, use at your own risk.
 */

#include <inttypes.h>

enum {
   TREE_OK,
   TREE_NODE_EXISTS,
   TREE_NODE_NOT_FOUND,
   TREE_OUT_OF_MEM,
   TREE_INVALID_ROTATION
};

typedef struct node_t{
   uint16_t key;
   //uint16_t aux; //user data
   struct node_t *left,*right, *p;
}tree_node;

typedef tree_node* node_ptr;

/** @brief Print out the current tree in ASCII. */
void print_tree(node_ptr starting_point);

/** @brief Delete all nodes in the tree. */
void clear_tree();

/** @brief Return a pointer to the root of the tree. */
node_ptr get_root();

/** @brief insert a new node into the tree.
 * @param key Value to insert into the tree
 * @return TREE_NODE_EXISTS if that key is already in the tree
 * @return TREE_OUT_OF_MEM if there is no more memory left
 * @return TREE_OK if the insertion was successful
*/
uint8_t insert(uint16_t key);

/** @brief Locate a particular value and return a pointer to that node
 * @param key Value to search for.
 * @return node_ptr Pointer to node if found, NULL if not
 */
node_ptr find(uint16_t key);

/** @brief Same as find. */
node_ptr find_node(uint16_t key);

/** @brief Delete a particular node from the tree.
 * @param key Node to delete from tree.
 * @return TREE_NODE_NOT_FOUND If the particular node wasn't found
 * @return TREE_OK if node was found and properly removed.
 */
uint8_t delete(uint16_t key);

/** @brief preform a left rotation on the node given by key
 * @param key Node to search for and rotate
 * @return TREE_NODE_NOT_FOUND If the particular key wasn't found.
 * @return TREE_INVALID_ROTATION If the rotation isn't possible.
 * @return TREE_OK If the rotation was successful.
 */
uint8_t left_rotate_node(uint16_t key);

/** @brief preform a right rotation on the node given by key
 * @param key Node to search for and rotate
 * @return TREE_NODE_NOT_FOUND If the particular key wasn't found.
 * @return TREE_INVALID_ROTATION If the rotation isn't possible.
 * @return TREE_OK If the rotation was successful.
 */
uint8_t right_rotate_node(uint16_t key);

/** @brief preform a left rotation on the node pointed to by x
 * @param x pointer to node to preform rotation on.
 * @return TREE_NODE_NOT_FOUND If the particular key wasn't found.
 * @return TREE_INVALID_ROTATION If the rotation isn't possible.
 * @return TREE_OK If the rotation was successful.
 */
uint8_t left_rotate(node_ptr x);

/** @brief preform a right rotation on the node pointed to by x
 * @param x pointer to node to preform rotation on.
 * @return TREE_NODE_NOT_FOUND If the particular key wasn't found.
 * @return TREE_INVALID_ROTATION If the rotation isn't possible.
 * @return TREE_OK If the rotation was successful.
 */
uint8_t right_rotate(node_ptr x);

/** @brief Splay the node pointed to by x
 * @param x Pointer to node to be splayed.
 */
void splay(node_ptr x);

/** @brief Search for a node and splay it.
 * @param key Node to search for and splay.
 */
void splay_node(uint16_t key);

/** @brief get a pointer to the node with the lowest value */
node_ptr get_min();

/** @brief get a pointer to the node with the highest value */
node_ptr get_max();

/** @brief count the number of nodes in the tree
 * @param starting_point Root of subtree to count nodes of.
 * @return Number of nodes in specified subtree.
 */
uint16_t count_nodes(node_ptr starting_point);

/** @brief Total up all of the keys in the tree.
 *
 * Note: the return value is only 16 bits and will
 * most likely overflow on a decent sized tree.
 */
uint16_t get_total(node_ptr starting_point);
