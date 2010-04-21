//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <time.h>

#include "mos.h"
#include "cortex.h"
#include "gui_xml.h"
#include "gui_topo.h"

#ifndef LIBXML_READER_ENABLED
#error "You need libxml2.6+ with XML Reader enabled"
#endif

static xmlDocPtr doc;

extern gui_topo_t *topo;

#ifndef MINI_GUI
extern GnomeCanvas *canvas;
#endif

static void print_xml_node (xmlNodePtr parent_node)
{
   xmlNodePtr node = parent_node;
   while (node != NULL) {
      if (node->type == XML_ELEMENT_NODE)
	 printf ("%s %s\n", node->name, xmlNodeGetContent (node));
      node = node->next;
   }
}

static void print_xml_doc ()
{
   gint i;

   xmlChar *xpath = XML_CHAR("//sensor_network/sensor_node");
   xmlXPathObjectPtr xpath_results;
   xpath_results = gui_xml_run_xpath (xpath);
   if (xpath_results) {
      xmlNodeSetPtr node_set = xpath_results->nodesetval;
      for (i = 0; i < node_set->nodeNr; i++) {
	 xmlNodePtr node = node_set->nodeTab[i]->xmlChildrenNode;
	 printf ("%s\n", node_set->nodeTab[i]->name);
	 print_xml_node (node);
      }
      xmlXPathFreeObject (xpath_results);
      goto cleanup;
   } else {
      debug ("Could not find any sensor nodes");
      goto cleanup;
   }

cleanup:
   //xmlFreeDoc (xml_doc);
   xmlCleanupParser ();
}

xmlDocPtr gui_xml_read_file (gchar *filename)
{
   doc = xmlParseFile (filename);
   if (doc == NULL) {
      debug ("Couldn't parse '%s'", filename);
      return NULL;
   }
   return doc;
}

static void gui_xml_write_file_helper (gpointer key, gpointer data,
				       gpointer user_data)
{
   topo_node_t *node = (topo_node_t *)data;
   xmlNodePtr root_node = (xmlNodePtr)user_data;
   gchar buf[256];
   
   if (g_hash_table_lookup (node->gui_topo->unconf_nodes,
			    GINT_TO_POINTER (node->id)) == NULL) {
      xmlNodePtr xml_node = xmlNewChild (root_node, NULL, XML_CHAR("sensor_node"), NULL);
      snprintf (buf, sizeof (buf), "%s", node->id);
      xmlNewChild (xml_node, NULL, XML_CHAR("node_id"), XML_CHAR(buf));
      snprintf (buf, sizeof (buf), "%.2f", node->x);
      xmlNewChild (xml_node, NULL, XML_CHAR("x"), XML_CHAR(buf));
      snprintf (buf, sizeof (buf), "%.2f", node->y);
      xmlNewChild (xml_node, NULL, XML_CHAR("y"), XML_CHAR(buf));
      snprintf (buf, sizeof (buf), "%.2f", node->z);
      xmlNewChild (xml_node, NULL, XML_CHAR("z"), XML_CHAR(buf));
   } else {
      debug("got unconf node");
   }
}

void gui_xml_write_file (gui_topo_t *topo, gchar *filename)
{
   doc = xmlNewDoc (XML_CHAR("1.0"));
   xmlNodePtr sensor_network = xmlNewNode (NULL, XML_CHAR("sensor_network"));
   xmlDocSetRootElement (doc, sensor_network);

   g_hash_table_foreach (topo->nodes, gui_xml_write_file_helper, sensor_network);

   debug ("Saving XML file to '%s'", filename);
   xmlSaveFormatFileEnc (filename, doc, "UTF-8", 1);
   xmlFreeDoc (doc);
   xmlCleanupParser ();
}

static void gui_xml_open_file_helper (xmlNodePtr parent_node)
{
   xmlNodePtr node = parent_node;
   gchar *node_id = (gchar *)NULL;
   gint x = -1;
   gint y = -1;
   gint z = -1;

   while (node != NULL) {
      if (node->type == XML_ELEMENT_NODE) {
	 if (!xmlStrcmp (node->name, XML_CHAR("node_id"))) {
	    node_id =  G_CHAR(xmlNodeGetContent (node));
	 }
	 if (!xmlStrcmp (node->name, XML_CHAR("x"))) {
	    x = atoi(G_CHAR((xmlNodeGetContent (node))));
	 }
	 if (!xmlStrcmp (node->name, XML_CHAR("y"))) {
	    y = atoi(G_CHAR((xmlNodeGetContent (node))));
	 }
	 if (!xmlStrcmp (node->name, XML_CHAR("z"))) {
	    z = atoi(G_CHAR((xmlNodeGetContent (node))));
	 }
      }
      node = node->next;
   }

   if (node_id == NULL || x == -1 || y == -1 || z == -1) {
      debug ("not a complete xml file?: %s %d %d %d",
	     node_id, x, y, z);
   }

   topo_node_t *topo_node = topo_node_from_id(node_id);

   if (1) {
      debug("need to be able to lookup nodes here");
      gui_topo_node_update_pos (topo_node, x, y, z);
   } else {
      gui_topo_node_new_pos (topo_node, (gdouble)x, (gdouble)y, (gdouble)z);
      debug("need to add something to a treeview");
   }
   
}

void gui_xml_open_file (gui_topo_t *topo, gchar *filename)
{
   doc = gui_xml_read_file (filename);
   gint i;
   
   xmlXPathObjectPtr xpath_results;
   xpath_results = gui_xml_run_xpath (XML_CHAR("//sensor_network/sensor_node"));
   if (xpath_results) {
      xmlNodeSetPtr node_set = xpath_results->nodesetval;
      for (i = 0; i < node_set->nodeNr; i++) {
	 xmlNodePtr node = node_set->nodeTab[i]->xmlChildrenNode;
	 gui_xml_open_file_helper (node);
      }
   } else {
      debug ("Could not find any sensor nodes");
   }

#ifndef MINI_GUI
   gnome_canvas_update_now (GNOME_CANVAS (topo->widget));
#endif
   
   xmlFreeDoc (doc);
   xmlCleanupParser ();
}

xmlXPathObjectPtr gui_xml_run_xpath (xmlChar *xpath_request)
{
   xmlXPathObjectPtr xpath_results;
   xmlXPathContextPtr context;

   context = xmlXPathNewContext (doc);
   xpath_results = xmlXPathEvalExpression (xpath_request, context);
   if (!xpath_results) {
      debug ("XPath expression '%s' did not return anything", xpath_request);
      xmlXPathFreeContext (context);
      return NULL;
   }
   if (xmlXPathNodeSetIsEmpty (xpath_results->nodesetval)) {
      debug ("No result found for XPath expr '%s'", xpath_request);
      xmlXPathFreeContext (context);
      return NULL;
   }

   xmlXPathFreeContext (context);
   return xpath_results;
}

