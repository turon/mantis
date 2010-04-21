//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "run.h"
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h> 
#include <stdbool.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <stdio.h>

void parse_function(char *input, char *delim, char ***argv, int *argc);
void get_paths(char ***paths, int *argc);
char *get_full_path(char *cmd);
bool check_file_perm(char *filename);

bool run_command(char *command)
{
   char *parse_keys = " \t\n";
   char *full_path;
   char **args;
   int argc=0;
   pid_t pid;

   parse_function(command, parse_keys, &args, &argc);

   if(args[0] == NULL)
      return false;

   
   full_path = get_full_path(args[0]);

   if(full_path == NULL){
      printf("Error: [%s] not found!", args[0]);
      free(args);
      return false;
   }
      
   
   free(args[0]);
   args[0] = full_path;

  pid = vfork();
  if(pid == 0)
     execv(args[0],args);
  else
     waitpid(pid, NULL, 0);

  free(args);
  return true;
}

char *get_full_path(char *cmd)
{
//parse the path variable
   char **paths;
   int pathc=0;
   int i=0;
   
   get_paths(&paths, &pathc);

   
  char *tmp_cmd=NULL;
  //check for absolute path
  if(cmd[0] == '/' || 
     strstr(cmd,"./") == cmd || 
     strstr(cmd,"../") == cmd){
    if(check_file_perm(cmd)){ //check perms
      tmp_cmd = strdup(cmd);
      return tmp_cmd;
    }
    else {
      return NULL;
    }
  } else { //relative path
     for(i=0;i<pathc;i++){
     //get some memory
     int tmp_size = strlen(paths[i]);
     int cmd_size = strlen(cmd);
     if(tmp_cmd) //free unused memory
       free(tmp_cmd);
     tmp_cmd = (char *)malloc(tmp_size + cmd_size  + 2); //1 for null 1 for '/'
     memcpy(tmp_cmd, paths[i], tmp_size); //copy in path 
     memcpy(tmp_cmd + tmp_size, "/",1);      //copy a slash
     memcpy(tmp_cmd + tmp_size + 1,cmd,cmd_size+1);//copy program
     int ret_val = check_file_perm(tmp_cmd);
    if(ret_val){ //check perms
      return tmp_cmd;
    }
   }
  }    //if we get here everything failed...
  if(tmp_cmd)
    free(tmp_cmd);
  tmp_cmd = NULL;
  return NULL;
}


void get_paths(char ***paths, int *argc)
{
   parse_function(getenv("PATH"), ":", paths, argc);
}

bool check_file_perm(char *filename)
{
  struct stat statbuf;
  int retval = stat(filename, &statbuf);
  if(!retval){
    if(statbuf.st_uid == getuid() && 
       (S_IXUSR & statbuf.st_mode)){
      return true;
    } else if(statbuf.st_gid == getgid() &&
	    (S_IXGRP & statbuf.st_mode)){
      return true;
    } else if(S_IXOTH & statbuf.st_mode){
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool file_exists(char *filename)
{
   struct stat statbuf;
   int retval = stat(filename, &statbuf);
   if(!retval)
      if(S_IFREG & statbuf.st_mode)
	 return true;

   return false;
}

void parse_function(char *input, char *delim, char ***argv, int *argc)
{
   char *start = strdup(input);
   char *pch = start;
   int letter_count;
   int i=0;

   *argc=0;
   
   letter_count = strcspn (pch, delim);
   while (*pch != '\0')
   {
      if(strlen(pch) == 0) break;
      (*argc)++;
      pch += letter_count; //go past letters
      letter_count = strspn (pch, delim);
      pch += letter_count; //go past space
      letter_count = strcspn (pch, delim);
   }

   *argv = malloc(sizeof(char *) * (*argc + 1));

   pch = start;
   letter_count = strcspn (pch, delim);
   while (*pch != '\0')
   {
      (*argv)[i] = calloc(letter_count+1,sizeof(char));
      strncpy((*argv)[i++], pch, letter_count);
      pch += letter_count; //go past letters
      letter_count = strspn (pch, delim);
      pch += letter_count; //go past space
      letter_count = strcspn (pch, delim);
   }
   (*argv)[i++] = 0;

   if(start != NULL){
      free(start);
   }
}
