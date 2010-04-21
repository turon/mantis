#include "mos.h"
#include "msched.h"
#include "uart.h"

#include "printf.h"
#include "plat_dep.h"
#include "sem.h"
#include "mutex.h"
#include "mem.h"


int count_test=0;

//Code below is for checking the minimum stack size
uint16_t actual_stack_size = 0;
mos_sem_t sem_min_stack_size;

void t1(void)
{
  actual_stack_size = mos_thread_current()->stackSize;

  mos_sem_post(&sem_min_stack_size);
  mos_thread_exit();
}

//Code below is for finding the maximum number of threads that system can handle
int numofthreads_flag=0;
mos_mutex_t mux_max_threads;

void t2(void)
{
  while (1) 
  {
    mos_mutex_lock(&mux_max_threads);
    if(numofthreads_flag == 0)
    {   
      mos_mutex_unlock(&mux_max_threads);
      break;
    }
    mos_mutex_unlock(&mux_max_threads);
  }  
  //printf("\nthread exiting\n");
  mos_thread_exit(); 
}

//Code below is for checking the valid priority level
int priority_flag = 0;
void t3(void)
{
  uint32_t sleep_time_1 = 25;
  int i =0;   
 
   for(i=0; i<20; i++) 
   {
      printf(".");
   }

   mos_thread_sleep(sleep_time_1);
   priority_flag=1;
   mos_thread_exit();
}

//code below is for preemption check
int tracker =0;
int PRIORITY_T1=0;
int PRIORITY_T2=2;
int PRIORITY_T3=1;

int sequence[5];
int sequence_index=0;
int combination_index=0; //Testing preemption for all possible combinations of 2 ,1 and 0 priority levels for three threads
//combination_index=0: PRIORITY_T1=2; PRIORITY_T2=1; PRIORITY_T3=0
//combination_index=1: PRIORITY_T1=2; PRIORITY_T2=0; PRIORITY_T3=1
//combination_index=2: PRIORITY_T1=1; PRIORITY_T2=0; PRIORITY_T3=2
//combination_index=3: PRIORITY_T1=1; PRIORITY_T2=2; PRIORITY_T3=0
//combination_index=4: PRIORITY_T1=0; PRIORITY_T2=1; PRIORITY_T3=2
//combination_index=5: PRIORITY_T1=0; PRIORITY_T2=2; PRIORITY_T3=1

//sequence_verify has first dimension for combination_index and 2nd for actual sequence for each combination of priorities
int sequence_verify[6][5] = {{1,2,3,5,6}, {1,2,5,3,6}, {1,2,5,6,3}, {1,6,2,3,5},{1,6,2,5,3},{1,6,2,3,5}};

void T3(void)
{
  tracker=3;

  sequence[sequence_index]=tracker;
  ++sequence_index;
}

void T2(void)
{
  tracker=2;
  
  sequence[sequence_index]=tracker;
  ++sequence_index;

  mos_thread_new(T3, 128, PRIORITY_T3);  

  tracker=5;

  sequence[sequence_index]=tracker;
  ++sequence_index;
}

void T1(void)
{
  tracker=1;

  sequence[sequence_index]=tracker;
  ++sequence_index;

  mos_thread_new(T2, 128, PRIORITY_T2); 
  
  tracker=6;
  sequence[sequence_index]=tracker;
  ++sequence_index;
}



void init_sequence(void)
{
   int k =0;
   for(k=0; k<5; k++)
   {
     sequence[k]=0;
   }
}

void set_priorities(void)
{
     switch(combination_index)
     {
       case 0:
         PRIORITY_T1=2;
         PRIORITY_T2=1;
         PRIORITY_T3=0;
         break;
       case 1:
         PRIORITY_T1=2;
         PRIORITY_T2=0;
         PRIORITY_T3=1;
         break;
       case 2:
         PRIORITY_T1=1;
         PRIORITY_T2=0;
         PRIORITY_T3=2;
         break;
       case 3:
         PRIORITY_T1=1;
         PRIORITY_T2=2;
         PRIORITY_T3=0;
         break;
       case 4:
         PRIORITY_T1=0;
         PRIORITY_T2=1;
         PRIORITY_T3=2;
         break;
       case 5:
         PRIORITY_T1=0;
         PRIORITY_T2=2;
         PRIORITY_T3=1;
         break;
     }
}

boolean verify_sequence(int combination)
{
   int j=0;
   //printf("\nsequence = ");

   for(j=0; j<5; j++)
   {
     //printf("%d\t", sequence[j]);
 

     if(sequence_verify[combination][j] != sequence[j])
     {
       return false;
     }

   }
   //printf("\n");
   return true;
}

//Code below is to check synchronization by implementing solution to first reader-writers problem
int shared_data =0;

int reader_flag=0;
int writer_flag=0;
int error_flag=0;

mos_sem_t mux_reader_writer;
mos_sem_t wrt_reader_writer;

int reader_count =0;
int writer_count =0;
int active_reader =0;

void writer(void)
{
  int item;
  int count =0;
  
  //printf("\nwriter =%d", writer_flag);

  while(count!=1000)
  {
    if(error_flag != 0)
    {
      mos_thread_exit();
    }

    item=(uint16_t)rand();

    mos_sem_wait(&wrt_reader_writer);
 
    ++writer_count;
    //printf("\ncurrent writers active = %d", writer_count);
    //printf("\ncurrent readers active =%d", active_reader);

    if((writer_count >1) || (active_reader != 0))
    {
      printf("\nError in synchronization.");
      error_flag=1;
      mos_thread_exit();  
    }

    shared_data = item;

    count++;

    //printf("\nitem written = %d", shared_data);
    
    --writer_count;

    mos_sem_post(&wrt_reader_writer);
  }
  //printf("\nwriter exit....\n");
  --writer_flag;
  //printf("\nwriter =%d", writer_flag);
}

void reader(void)
{
  int item;
  int count =0;

  //printf("\nreader =%d", reader_flag);

  while(count!=1000)
  {
    mos_sem_wait(&mux_reader_writer);
    ++reader_count;

    if(reader_count == 1)
     mos_sem_wait(&wrt_reader_writer);

    ++active_reader;

    //printf("\ncurrent writers active = %d", writer_count);
    //printf("\ncurrent readers active =%d", active_reader);

    mos_sem_post(&mux_reader_writer);

    if(writer_count > 0)
    {
      printf("\nError in synchronization.");
      error_flag=1;
      mos_thread_exit();  
    }
    
    item = shared_data;

    count++;

    //printf("\nitem read = %d", shared_data);

    mos_sem_wait(&mux_reader_writer);
    --reader_count;

    --active_reader;

    if(reader_count == 0)
      mos_sem_post(&wrt_reader_writer);
    mos_sem_post(&mux_reader_writer);

  }
  //printf("\nReader exit......");
  --reader_flag;
  //printf("\nreader =%d", reader_flag);
}

//Code below is for dining philosopher's problem using semaphores
#define NUM_PHILOS 4
#define LEFT (i+NUM_PHILOS-1)%NUM_PHILOS
#define RIGHT (i+1)%NUM_PHILOS
#define THINKING 0
#define HUNGRY 1
#define EATING 2

int state[NUM_PHILOS];
int philosopher_count=0;
int someone_eating = 0;
int last_state[NUM_PHILOS] = {0,0,0,0};
boolean loop_continue;

int num_of_eat[NUM_PHILOS] = {0,0,0,0};
mos_mutex_t mux_num_of_eat;

mos_sem_t mux_dining_phil1;
mos_sem_t s_dining_phil[NUM_PHILOS];

void test(int i)
{
  if((state[i] == HUNGRY) && (state[LEFT]!= EATING) && (state[RIGHT] != EATING)) 
  {
    if(someone_eating < 0 || someone_eating >=2)
    {
      printf("\nsynchronization error");  
      error_flag=1;    
    }
    else
    {
      state[i]=EATING;

      mos_mutex_lock(&mux_num_of_eat);
      ++num_of_eat[i];
      mos_mutex_unlock(&mux_num_of_eat);

      if(last_state[i] != HUNGRY)
      {
        printf("\nchange in error to eating");
        error_flag=1;
      }
      last_state[i]=EATING;
    
      ++someone_eating;
      //printf("%C%Ceat.....", i, state[i]);
      //printf("\nsomeone_eating = %d", someone_eating);
    }
    mos_sem_post(&s_dining_phil[i]);
  }
}

void take_forks(int i)
{ 
  mos_sem_wait(&mux_dining_phil1);
  state[i] = HUNGRY;
  if(last_state[i] != THINKING)
  {
    printf("\nerror in change to hungry");
    error_flag=1;
  }
  last_state[i] = HUNGRY;
  //printf("%C%Chungry..", i,state[i]);

  test(i);
  mos_sem_post(&mux_dining_phil1);
  mos_sem_wait(&s_dining_phil[i]);
}

void put_forks(int i)
{ 
  mos_sem_wait(&mux_dining_phil1);
  state[i] = THINKING;

  if(last_state[i] != EATING)
  {
    printf("\nerror in change to thinking");
    error_flag=1;
  }
  last_state[i] = THINKING;

  //printf("%C%Cthink...", i,state[i]);

  --someone_eating;

  test(LEFT);
  test(RIGHT);
  mos_sem_post(&mux_dining_phil1);
}

void philosopher1()
{
  int i = philosopher_count;

  //printf("\nPhilosopher count is now \n%d", philosopher_count);
  //printf("%C%Cthink...", i,state[i]);

  while(loop_continue == true)
  {
    //think();
    //printf("%C%Cthink", i,state[i]);
    printf(".");

    take_forks(i);
    //state[i]=EATING;
    //printf("\n%C%Ceat....", i, state[i]);

    put_forks(i);
  }
  --philosopher_count;
  //printf("\nPhilosopher count is now %d\n", philosopher_count);
}

//code below is for dining philosopher's problem using mutexes
mos_mutex_t mux;

int num_of_eat2[N] = {0,0,0,0};
mos_mutex_t mux_num_of_eat2;

void test2(int i)
{
  if((state[i] == HUNGRY) && (state[LEFT]!= EATING) && (state[RIGHT] != EATING)) 
  {
    if(someone_eating < 0 || someone_eating >=2)
    {
      printf("\nsynchronization error");    
      error_flag=1;   
    }
    else
    {
      state[i]=EATING;

      mos_mutex_lock(&mux_num_of_eat2);
      ++num_of_eat2[i];
      mos_mutex_unlock(&mux_num_of_eat2);

      if(last_state[i] != HUNGRY)
      {
        printf("\nchange in error to eating");
        error_flag=1; 
      }
      last_state[i]=EATING;
    
      ++someone_eating;
      //printf("\n%Ceat", i);
      //printf("\nsomeone_eating = %d", someone_eating);
    }
    mos_sem_post(&s_dining_phil[i]);
  }
}

void put_forks2(int i)
{ 
  mos_mutex_lock(&mux);

  state[i] = THINKING;

  if(last_state[i] != EATING)
  {
    printf("\nerror in change to thinking");
    error_flag=1; 
  }
  last_state[i] = THINKING;

  //printf("\n%Cthink", i);
  --someone_eating;

  test2(LEFT);
  test2(RIGHT);
  mos_mutex_unlock(&mux);
}


void take_forks2(int i)
{ 
  mos_mutex_lock(&mux);
  state[i] = HUNGRY;
  if(last_state[i] != THINKING)
  {
    printf("\nerror in change to hungry");
    error_flag=1; 
  }
  last_state[i] = HUNGRY;
  //printf("\n%Chungry", i);

  test2(i);
  mos_mutex_unlock(&mux);
  mos_sem_wait(&s_dining_phil[i]);
}

void philosopher2()
{
  int i = philosopher_count;

  //printf("\nPhilosopher count is now \n%d", philosopher_count);
  //printf("\n%CthinkStart...", i);

  while(loop_continue == true)
  {
    //think();
    printf(".");

    take_forks2(i);

    put_forks2(i);
  }
  --philosopher_count;
  //printf("\nPhilosopher count is now %d\n", philosopher_count);
}

void start(void)
{
   int rc =0;

   printf("\n*****************************************************************************************************");
   printf("\n\tThis test-app tests the thread module.");
   printf("\n\tThere will be 8 tests in all.");
   printf("\n\tBefore starting any test, you will be told which test is about to run.");
   printf("\n\tIf the program hangs at any point, there should be some problem in the current test.");
   printf("\n\tAfter the successful completion of all tests, you should get success message.");
   printf("\n*****************************************************************************************************\n\n");

   ////////////Test to find the available memory on the system/////////////
   uint16_t s = 64;
   uint16_t mem = 0;
   void *mem_block;
   int count=0;
   
   printf("\nChecking for free memory available.....\n");   
   printf("This test gives an idea of upper bound on the total stack size that can be allocated among al the threads.\nTest tries to allocate blocks (one at a time) with increasing sizes of memory. First allocation starts at 64 bytes. If the block is successfully allocated, it means that there was sufficient memory available. That block is then deallocated. These steps of allocation and deallocation are repeated. For each successive allocation, memory is doubled until there is a failure in allocation. For the next run, last successful memory allocation is used as base and starting increments from 64 bytes, allocation and deallocation is done again, doubling the increment size each time until a failure is encountered again. Process is repeated until the point when for any run first increment of 64 bytes fails. So, the final output is the memory available on the system (within a margin of 64 bytes) after this test-app is loaded\n\n");

   while(1)
   {

     //printf("checking for memory size = %l\n", mem+s);   
     
     if((mem_block = mos_mem_alloc(mem+s)) != NULL)
     {
       mos_mem_free(mem_block);
       count++;

       s=s*2;   
     }
     else
     {
       if(count == 0)
       {
         printf("Free space available on system is somewhere in between %d and %d bytes.\n\n\n\n", mem,mem+s);
         ++count_test;
         break;
       }
       else
       {
         //printf("Failed to allocate memory of size %d\n", mem+s);
         count=0;
         mem=mem+(s/2);
         s=64;
       }       
     }
    }


   ////////Test to find the minimum size of the stack with which any thread is created///////
   int stack_size=256;
   int dec = 32;
   
   rc =0;   

   mos_sem_init(&sem_min_stack_size, 0);
   
   printf("Checking for minimum stack size with which any thread can be created......\n");
   printf("In the current system, threads can be allocated with some minimum stack sizes. If the threads are created with the stack size below these limit, operating system will adjust the stack size for that thread to the minimum stack size that is valid on that node. On the telosb node, you should get 128 bytes, on mica2 and micaz, it should be 64 bytes.\nTest starts creating threads one at a time with decreasing stack sizes starting from 256 bytes. When the thread is created, actual stack size of the created thread is compared to the size that was given while crating the thread. If both match, it means that the stack size is not below minimum in which case we terminate that thread and create another with stack size of 32 bytes lesser than the previous one. When the actual stack size of the created thread is more than the size that was given while creating, it means that the thread size given was below the minimum and so the stack size from the previous run is the minimum stack size.\n\n");
   while(1)
   {
     actual_stack_size = 0;
     rc = mos_thread_new(t1, stack_size, PRIORITY_NORMAL);

     if(rc == 0)
     {
       stack_size-=dec; //thread successfully created.
     }
     else
     {
       printf("Thread is not created due to insufficient memory or insufficient number of threads.\n\n");
       break;
     } 
 
     mos_sem_wait(&sem_min_stack_size);

     if((stack_size + dec) < actual_stack_size)
     {
       printf("Minimum stack size for any thread on this system is %d bytes.\n\n\n\n", actual_stack_size);
       ++count_test;
       break;
     }
   }

    //////////////////Test to find the maximum number of threads that system can handle//////////
   int count_maxthreads=0;
   rc=1;

   mos_mutex_init(&mux_max_threads);
 
   printf("Checking the number of application threads that system can handle.......\n");
   printf("Test keeps creating threads and also keeping a count of the number of threads created. If an error code indicating the fact that no more threads can be created, is returned while creating a thread, we get a limit on the maximum number of application threads.\n\n");
   for(count_maxthreads=0; count_maxthreads<10; count_maxthreads++)
   {
     mos_mutex_lock(&mux_max_threads);

     //create new thread
     if((rc = mos_thread_new (t2, 128, PRIORITY_NORMAL)) == 1)
     {
       printf("System can handle only %d application threads.\n\n\n\n", count_maxthreads);
       numofthreads_flag = 0;
       mos_mutex_unlock(&mux_max_threads);
       ++count_test;
       break;
     }
     else if(rc ==0)
     {
       numofthreads_flag++;
       mos_mutex_unlock(&mux_max_threads);
     }
   }


    //////////Test to check the valid priority levels/////////

   int priority_start = 5;
   int priority = priority_start;
   int invalid_count =0;
   uint32_t sleep_time_start = 100;  

   rc = 0;
   printf("Checking for valid priority levels........\n");
   printf("Test tries to create threads (one at a time) of different priorities starting from priority level 4 and then first checking for all the priority levels below 4 until it gets 4 invalid priority levels. After this, all priority levels above 4 are checked until 4 invalid priority levels are found. Invalidity of a priority level is known from the error code returned when thread creation fails. Also, there is implementation to catch if a thread is created with invalid priority level (though it should not happen as it has been fixed now). Currently sytem supports 5 priority levels: 0 through 4. You should get this as result.\n");

   //This while loop keeps checking whether priority levels "priority_start-1" and each level below that are valid or not. It keeps checking until it gets four invalid prioirty levels.
   while(1)
   {
     priority--;
     //printf("checking for priority = %d......", priority);

     rc = mos_thread_new (t3, 128, priority);
 
     if(rc == 3)
     {
       if(priority < 0)
         printf("Priority level %d is invalid (-ve priority but doesn't print as is, because printf does not support negative signed numbers). \n", priority);
       else
         printf("Priority level %d is invalid. \n", priority);

       
       priority_flag = 0;

       if(invalid_count ==3)
         break;  
       invalid_count++;
     }

     if(rc == 0)
     {
       priority_flag = 0; 
     }
     else if((rc == 1) || (rc == 2))
     {
       printf("There is either memory issue or number of threads exceed the maximum limit.\n");
       break;
     }
     mos_thread_sleep(sleep_time_start);

     if((priority_flag == 0) && (rc==0))
     {
       printf("Thread with priority = %d doesn't work.\n", priority);
       break;
     }
     if(priority_flag == 1)
     {
        printf("Priority level %d is valid.\n", priority);
     }
     if((rc == 3) && (priority_flag ==1))
     {
       printf("Invalid priority but thread still created.\n");
     }
   }
   
   //This while loop keeps checking whether priority levels "priority_start" and each level above that are valid or not. It keeps checking until it gets four invalid prioirty levels.
   priority =priority_start-1;
   invalid_count =0;

   rc =0;

   while(1)
   {
     priority++;

     //printf("checking for priority = %d........", priority);
     rc = mos_thread_new (t3, 128, priority);

     //printf("rc = %d\n", rc);

     if(rc == 3)
     {
       //printf("Priority level %d is invalid\n", priority);
       if(priority < 0)
         printf("Priority level %d is invalid (-ve priority but doesn't print as is, because printf does not support negative signed numbers). \n", priority);
       else
         printf("Priority level %d is invalid. \n", priority);
       
       priority_flag = 0;

       if(invalid_count ==3)
         break;  
       invalid_count++;
     }

     if(rc == 0)
     {
       priority_flag = 0; 
     }
     else if((rc == 1) || (rc == 2))
     {
       printf("There is either memory issue or number of threads exceed the maximum limit.\n");
       break;
     }
     mos_thread_sleep(sleep_time_start);

     if((priority_flag == 0) && (rc==0))
     {
       printf("Thread with priority = %d doesn't work.\n", priority);
       break;
     }
     if(priority_flag == 1)
     {
        printf("Priority level %d is valid.\n", priority);
     }
     if((rc == 3) && (priority_flag ==1))
     {
       printf("Invalid priority but thread still created.\n");
     }
   }
   printf("\n\n");
   ++count_test;

   /////////Test to verify preemption/////////
   sequence_index=0;
   boolean rc_preempt = true;
   
   printf("\nChecking whether or not OS supports preemptive scheduling of threads.........\n");
   printf("\n.In this test, 3 threads are created: T1, T2 and T3. Main thread i.e. start() creates T1. T1 creates T2 and T2 creates T3. Priorities of each of these threads are higher than priority of the start thread which is PRIORITY_NORMAL by default. As there are three priority levels above PRIORITY_NORMAL, these priority levels are selected for T1, T2 and T3. There are 6 possible combinations with different priorities for each thread. Expected sequence of execution for each of these combinations is hard coded in the logic. The actual sequence of execution for each combination is then compared to the expected one for that combination and if these don't match, an error message will be printed out giving the priorities of each thread in the current run for which test failed. Otherwise, a success message is printed at the end.\n");

   for(combination_index=0; combination_index<6; combination_index++)
   {
     sequence_index=0;
     tracker =0;

     set_priorities();
     init_sequence();

     mos_thread_new(T1, 128, PRIORITY_T1); 

     //printf("Tracker = %d\n", tracker);

     rc_preempt = verify_sequence(combination_index);

     if(rc_preempt == false)
     {
       printf("\nPreemption failed for the case when priority of T1 = %d, T2 = %d, T3 = %d.\n\n\n", PRIORITY_T1, PRIORITY_T2, PRIORITY_T3);
     }
     else if(combination_index == 5)
     {
       printf("\nEach of the six test cases to check preemption was successful. OS supports preemptive scheduling of threads.\n\n\n");
       ++count_test;
     }
   }

   /////////////Test to check semaphore by modeling reader writer problem////////////////
   printf("\nChecking thread synchronization using semaphore by modeling reader-writer problem......");
   printf("\n\nTest models the solution to first reader-writers problem. Two reader and two writer threads are created. Writers write into a shared variable and readers read from that variable for 1000 times. Semaphores are used to synchronize threads. Whenever there is an active writer a check is made to make sure only one writer is active and no reader is active. Similarly when a reader is reading, it is checked that no writer is active. If these conditions are not satisfied, you will get an error message.\n");

  mos_sem_init(&mux_reader_writer,1);
  mos_sem_init(&wrt_reader_writer,1);

  ++writer_flag;
  mos_thread_new(writer, 128, PRIORITY_NORMAL);

  ++writer_flag;
  mos_thread_new(writer, 128, PRIORITY_NORMAL);

  ++reader_flag;
  mos_thread_new(reader, 128, PRIORITY_NORMAL);

  ++reader_flag;
  mos_thread_new(reader, 128, PRIORITY_NORMAL);


  while(1)
  {
    if(error_flag == 1)
    {
      printf("\nError in synchronization.");
      break;
    }

    if((reader_flag == 0) && (writer_flag == 0))
    {
      printf("\nReader & writer threads are properly synchronized.\n\n\n");
      ++count_test;
      break;
    }
    else 
    {
      printf(".");
    }
  }

   /////////////Test to check semaphore by modeling dining philosopher problem////////////////
  printf("\nChecking thread synchronization using semaphore by modeling dining philosopher problem......");
 printf("\n\nTest models the solution to dining philosopher's problem. Four philosopher threads are created. Each philosopher needs two forks, one on his left and one on his right. Semaphores are used to synchronize threads. There is a binary semaphore which protects the critical sections. Also, there are four semaphores, one for each of the philosophers.\nTest makes sure that state machine is maintained and also makes sure that the number of philosophers eating at any time is in either 1 or 2. If these conditions are not satisfied, you will get an error message. At the end of this test, number of times each philosopher entered the eating state will be printed.\n");

  int index=0;

  error_flag=0;

  loop_continue=true;
  mos_sem_init(&mux_dining_phil1, 1);
  mos_mutex_init(&mux_num_of_eat);  

  for(philosopher_count =0; philosopher_count < N; philosopher_count++)
  {
    mos_sem_init(&s_dining_phil[philosopher_count],0);
    state[philosopher_count] = THINKING;
  }
  philosopher_count = -1;
  ++philosopher_count;

  mos_thread_new(philosopher1, 128, PRIORITY_NORMAL);

  ++philosopher_count;
  mos_thread_new(philosopher1, 128, PRIORITY_NORMAL);

  ++philosopher_count;
  mos_thread_new(philosopher1, 128, PRIORITY_NORMAL);

  ++philosopher_count;
  mos_thread_new(philosopher1, 128, PRIORITY_NORMAL);

  mos_thread_sleep(2000);
  printf("\nTerminating threads....");
  loop_continue = false;

  mos_mutex_lock(&mux_num_of_eat);
  for(index=0; index<N; index++)
  {
    printf("\nPhilosopher %d ate %d times", index, num_of_eat[index]);
  }
  mos_mutex_unlock(&mux_num_of_eat);
  if(error_flag != 1)
  {
    printf("\n\nAll threads were synchronized properly.\n\n\n");
    count_test++;
  }
  else
  {
    printf("\nThere were synchronization errors.\n\n\n");
  }

   /////////////Test to check mutex locks by modeling dining philosopher problem////////////////
  printf("\nChecking thread synchronization using mutexes by modeling dining philosopher problem......");
  printf("\n\nTest models the solution to dining philosopher's problem. Four philosopher threads are created. Each philosopher needs two forks, one on his left and one on his right. Mutexes and semaphores are used to synchronize threads. There is a mutex lock which protects the critical sections. Also, there are four semaphores, one for each of the philosophers.\nTest makes sure that state machine is maintained and also makes sure that the number of philosophers eating at any time is in either 1 or 2. If these conditions are not satisfied, you will get an error message. At the end of this test, number of times each philosopher entered the eating state will be printed.\n");

  philosopher_count=0;
  someone_eating = 0;
  loop_continue = true;
  error_flag=0;

  for(index=0; index<N; index++)
    last_state[index] = 0;

  mos_mutex_init(&mux);
  mos_mutex_init(&mux_num_of_eat2); 

  for(philosopher_count =0; philosopher_count < N; philosopher_count++)
  {
    mos_sem_init(&s_dining_phil[philosopher_count],0);
    state[philosopher_count] = THINKING;
  }
  philosopher_count = -1;
  ++philosopher_count;

  mos_thread_new(philosopher2, 128, PRIORITY_NORMAL);

  ++philosopher_count;
  mos_thread_new(philosopher2, 128, PRIORITY_NORMAL);

  ++philosopher_count;
  mos_thread_new(philosopher2, 128, PRIORITY_NORMAL);

  ++philosopher_count;
  mos_thread_new(philosopher2, 128, PRIORITY_NORMAL);

  mos_thread_sleep(2000);
  printf("\nTerminating threads..\n");
  loop_continue = false;

  mos_mutex_lock(&mux_num_of_eat2);
  for(index=0; index<N; index++)
  {
    printf("\nPhilosopher %d ate %d times", index, num_of_eat2[index]);
  }
  mos_mutex_unlock(&mux_num_of_eat2);

  if(error_flag != 1)
  {
    printf("\n\nAll threads were synchronized properly.\n\n\n");
    count_test++;
  }
  else
  {
    printf("\nThere were synchronization errors.\n\n\n");
  }


   printf("\n*****************************************************************************************************"); 
   printf("\n\tTesting complete."); 
   printf("\n\t%d tests were successful.", count_test);
   printf("\n*****************************************************************************************************\n\n");

}





