//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "filter_model.h"
#include "cortex.h"
#include <stdlib.h>

#ifndef LIBXML_READER_ENABLED
#error "You need libxml2.6+ with XML Reader enabled"
#endif

static const xmlChar *root_node = (const xmlChar *)"bionet_filter_list";
static const xmlChar *filter_list = (const xmlChar *)"bionet_filter";
static xmlDocPtr doc;
static char *filename = "filter_list.xml";

// the list of filters held in memory
static GSList *filter_list_list;

static GSList *add_filter_list_funcs = NULL;
static GSList *remove_filter_list_funcs = NULL;

static void filter_show_list();
GSList *get_filter_list();

static gboolean filter_model_read_xml();
static gboolean create_default_filter_list();


static GSList *fill_constraints(xmlNodePtr bionet_filter_p);

void filter_model_add_filter_register_func(filter_model_add_filter_f func)
{
   add_filter_list_funcs = g_slist_append(add_filter_list_funcs, func);
}

void filter_model_remove_filter_register_func(filter_model_remove_filter_f func)
{
   remove_filter_list_funcs = g_slist_append(remove_filter_list_funcs, func);
}

void filter_model_add_filter_list(filter_list_t *filter_list)
{
   GSList *cur = add_filter_list_funcs;
   while(cur != NULL)
   {
      filter_model_add_filter_f func = cur->data;
      func(filter_list);
      cur = cur -> next;
   }
}

void filter_model_remove_filter_list(filter_list_t *filter_list)
{
   GSList *cur = remove_filter_list_funcs;
   while(cur != NULL)
   {
      filter_model_remove_filter_f func = cur->data;
      func(filter_list);
      cur = cur -> next;
   }
}

gboolean filter_model_init()
{
   filter_list_list = NULL;

   create_default_filter_list();
   
   gboolean ret = filter_model_read_xml();
   
   return ret;
}

static gboolean create_default_filter_list()
{
   char *name = "None";
   //alloc the memory for the default list
   filter_list_t *default_list = malloc(sizeof(filter_list_t));

   //set the name to be 
   default_list->name = malloc(strlen(name) + 1);
   strcpy(default_list->name, name);

   //setup the constraints
   default_list->constraint_list = NULL;

   constraint_t *constraints = malloc(sizeof(constraint_t));

   constraints->hab_type = malloc(strlen("*") + 1);
   constraints->hab_id   = malloc(strlen("*") + 1);
   constraints->node_id  = malloc(strlen("*") + 1);
   constraints->resource = malloc(strlen("*") + 1);

   strcpy(constraints->hab_type, "*");
   strcpy(constraints->hab_id,   "*");
   strcpy(constraints->node_id,  "*");
   strcpy(constraints->resource, "*");

   default_list->constraint_list = g_slist_append(default_list->constraint_list,
						  constraints);

   filter_list_list = g_slist_append(filter_list_list, default_list);

   return true;
}

static gboolean filter_model_read_xml()
{
   xmlNodePtr cur;
   
   doc = xmlParseFile(filename);
   
   if (doc == NULL ) {
      filter_model_save_filter_list();
      return false;
   }

   cur = xmlDocGetRootElement(doc);
   
   if (cur == NULL) {
      filter_model_save_filter_list();
      xmlFreeDoc(doc);
      return false;
   }
	
   if (xmlStrcmp(cur->name, root_node)) {   filter_model_show_list();

      debug("document of the wrong type, root node != bionet_filter_list");
      xmlFreeDoc(doc);
      return false;
   }

   cur = cur->xmlChildrenNode;
   while (cur != NULL) {
      if(!xmlStrcmp(cur->name, filter_list))
      {
	 xmlChar *filter_name;
	 filter_name = xmlGetProp(cur, (const xmlChar *)"name");
	 GSList *constraint_list = fill_constraints(cur);
	 
	 filter_model_update_filter_list(filter_name, constraint_list);
	 xmlFree(filter_name);
      }
      cur = cur -> next;
   }

   xmlFreeDoc(doc);

   return true;
}

filter_list_t *filter_model_find_filter_list(char *name)
{
   filter_list_t *ret;
   GSList *p = filter_list_list;
   while(p != NULL)
   {
      ret = (filter_list_t *)p->data;
      if(strcmp(ret->name,name) == 0)
	 break;
      p = p -> next;
   }
   
   if(p == NULL)
      return NULL;
   else
      return ret;

}

void filter_model_update_filter_list(char *name, GSList *constraint_list)
{
   filter_list_t *flist = filter_model_find_filter_list(name);

   if(flist != NULL) //found this list, edit it
   {
      g_slist_free(flist->constraint_list);
      flist->constraint_list = constraint_list;
   } else { //new filter list to add
      filter_list_t *new_filter_list;
      new_filter_list = (filter_list_t *)malloc(sizeof(filter_list_t));
      new_filter_list->name = (char *)malloc(strlen(name) + 1);
      strcpy(new_filter_list->name, name);
      new_filter_list->constraint_list = constraint_list;
      filter_list_list = g_slist_append(filter_list_list,
					new_filter_list);
      filter_model_add_filter_list(new_filter_list);
   }
}

void filter_model_delete_filter_list(char *name)
{
   filter_list_t *flist = filter_model_find_filter_list(name);

   if(flist == NULL)
      return;

   g_slist_free(flist->constraint_list);
   g_slist_remove(filter_list_list, flist);
}

void filter_model_show_list()
{
   GSList *cur = filter_list_list;
   while(cur != NULL) {
      filter_list_t *filter_list = (filter_list_t *)cur->data;
      printf(" Filter List Name: %s\n", filter_list->name);

      GSList *clist = filter_list->constraint_list;
      for(; clist != NULL; clist = clist->next)
      {
	 constraint_t *c = clist->data;
	 printf("    constraint:\t%s.%s.%s:%s\n",
		c->hab_type,
		c->hab_id,
		c->node_id,
		c->resource);
      }

      cur = cur->next;
   }
}

static void save_helper(gpointer data,
			gpointer user_data)
{
   filter_list_t *flist = (filter_list_t *)data;


   if(strcmp(flist->name, "None") == 0)
      return;
   
   xmlNodePtr root_node = (xmlNodePtr)user_data;

   xmlNodePtr new_node = xmlNewChild(root_node, NULL, (xmlChar *)"bionet_filter", NULL);
   xmlAttrPtr newattr = xmlNewProp(new_node, (xmlChar *)"name", flist->name);

   debug("saving constraints for: %s", flist->name);
   
   GSList *p = flist->constraint_list;
   while(p != NULL)
   {
      constraint_t *constraint = (constraint_t *)p->data;
      debug("constraints: %s.%s.%s:%s",
	     constraint->hab_type,
	     constraint->hab_id,
	     constraint->node_id,
	     constraint->resource);


      xmlNodePtr cur_constraint = xmlNewChild(new_node, NULL,
					      (xmlChar *)"bionet_constraint", NULL);
      
      xmlNewTextChild(cur_constraint, NULL,
		      (xmlChar *)"hab_type",
		      (xmlChar *)constraint->hab_type);
      xmlNewTextChild(cur_constraint, NULL,
		      (xmlChar *)"hab_id",
		      (xmlChar *)constraint->hab_id);
      xmlNewTextChild(cur_constraint, NULL,
		      (xmlChar *)"node_id",
		      (xmlChar *)constraint->node_id);
      xmlNewTextChild(cur_constraint, NULL,
		      (xmlChar *)"resource",
		      (xmlChar *)constraint->resource);
      p = p -> next;
   }
   
}

void filter_model_save_filter_list()
{
   doc = xmlNewDoc("1.0");
   xmlNodePtr cur = xmlNewNode(NULL, "bionet_filter_list");
   xmlDocSetRootElement(doc, cur);

   g_slist_foreach(filter_list_list, save_helper, (gpointer *)cur);
   
   xmlSaveFormatFileEnc(filename, doc, "UTF-8", 1);
   xmlFreeDoc(doc);
   xmlCleanupParser();
   
}

static GSList *fill_constraints(xmlNodePtr bionet_filter_p)
{
   GSList *constraint_list = NULL;
   xmlNodePtr cur_constraint = bionet_filter_p->xmlChildrenNode;
   while (cur_constraint != NULL) {
      if(!xmlStrcmp(cur_constraint->name, (const xmlChar *)"bionet_constraint"))
      {
	 constraint_t *new_constraint = malloc(sizeof(constraint_t));
	 xmlNodePtr cur_item = cur_constraint -> xmlChildrenNode;
	 while (cur_item != NULL) {
	    if(!xmlStrcmp(cur_item->name, (const xmlChar *)"hab_id"))
	       new_constraint->hab_id = xmlNodeListGetString(doc,
							     cur_item->xmlChildrenNode,
							     1);

	    if(!xmlStrcmp(cur_item->name, (const xmlChar *)"hab_type"))
	       new_constraint->hab_type = xmlNodeListGetString(doc,
							       cur_item->xmlChildrenNode,
							       1);

	    if(!xmlStrcmp(cur_item->name, (const xmlChar *)"node_id"))
	       new_constraint->node_id = xmlNodeListGetString(doc,
							      cur_item->xmlChildrenNode,
							      1);

	    if(!xmlStrcmp(cur_item->name, (const xmlChar *)"resource"))
	       new_constraint->resource = xmlNodeListGetString(doc,
							       cur_item->xmlChildrenNode,
							       1);
	    
	    cur_item = cur_item -> next;
	 }
	 constraint_list = g_slist_append(constraint_list,
					  new_constraint);

      }
      cur_constraint = cur_constraint -> next;
   }

   return constraint_list;
}

GSList *get_filter_list()
{
   return filter_list_list;
}
