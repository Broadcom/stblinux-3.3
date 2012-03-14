
using namespace std;

#include <unistd.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include "bonnie.h"
#ifdef HAVE_VECTOR
#include <vector>
#else
#include <vector.h>
#endif

// Read the specified number of megabytes of data from the fd and return the
// amount of time elapsed in seconds.
double readmegs(int fd, int size, char *buf);

// Returns the mean of the values in the array.  If the array contains
// more than 2 items then discard the highest and lowest thirds of the
// results before calculating the mean.
double average(double *array, int count);
void printavg(int position, double avg, int block_size);

const int meg = 1024*1024;
typedef double *PDOUBLE;

void usage()
{
  printf("Usage: zcav [-b block-size] [-c count]\n"
         "            [-u uid-to-use:gid-to-use] [-g gid-to-use]\n"
         "            [-f] file-name\n"
         "File name of \"-\" means standard input\n"
         "Count is the number of times to read the data (default 1).\n"
         "Version: " BON_VERSION "\n");
  exit(1);
}

int main(int argc, char *argv[])
{
  vector<double *> times;
  vector<int> count;
  int block_size = 100;

  int max_loops = 1;
  char *file_name = NULL;

  char *userName = NULL, *groupName = NULL;
  int c;
  while(-1 != (c = getopt(argc, argv, "-c:b:f:g:u:")) )
  {
    switch(char(c))
    {
      case 'b':
        block_size = atoi(optarg);
      break;
      case 'c':
        max_loops = atoi(optarg);
      break;
      case 'g':
        if(groupName)
          usage();
        groupName = optarg;
      break;
      case 'u':
      {
        if(userName)
          usage();
        userName = strdup(optarg);
        int i;
        for(i = 0; userName[i] && userName[i] != ':'; i++);
        if(userName[i] == ':')
        {
          if(groupName)
            usage();
          userName[i] = '\0';
          groupName = &userName[i + 1];
        }
      }
      break;
      case 'f':
      case char(1):
        file_name = optarg;
      break;
      default:
        usage();
    }
  }

  if(userName || groupName)
  {
    if(bon_setugid(userName, groupName, false))
      return 1;
    if(userName)
      free(userName);
  }

  if(max_loops < 1 || block_size < 1)
    usage();
  if(!file_name)
    usage();
  printf("#loops: %d, version: %s\n", max_loops, BON_VERSION);
  printf("#block K/s time\n");

  int i;
  char *buf = new char[meg];
  int fd;
  if(strcmp(file_name, "-"))
  {
    fd = open(file_name, O_RDONLY);
    if(fd == -1)
    {
      printf("Can't open %s\n", file_name);
      return 1;
    }
  }
  else
  {
    fd = 0;
  }
  if(max_loops > 1)
  {
    for(int loops = 0; loops < max_loops; loops++)
    {
      if(lseek(fd, 0, SEEK_SET))
      {
        printf("Can't llseek().\n");
        return 1;
      }
      for(i = 0; loops == 0 || times[0][i] != -1.0; i++)
      {
        double read_time = readmegs(fd, block_size, buf);
        if(loops == 0)
        {
          times.push_back(new double[max_loops]);
          count.push_back(0);
        }
        times[i][loops] = read_time;
        if(read_time < 0.0)
        {
          if(i == 0)
          {
            fprintf(stderr, "Input file too small.\n");
            return 1;
          }
          times[i][0] = -1.0;
          break;
        }
        count[i]++;
      }
      fprintf(stderr, "Finished loop %d.\n", loops + 1);
    }
    for(i = 0; count[i]; i++)
    {
      printavg(i, average(times[i], count[i]), block_size);
    }
  }
  else
  {
    for(i = 0; 1; i++)
    {
      double read_time = readmegs(fd, block_size, buf);
      if(read_time < 0.0)
        break;
      printavg(i, read_time, block_size);
    }
    if(i == 0)
    {
      fprintf(stderr, "Input file too small.\n");
      return 1;
    }
  }
  return 0;
}

void printavg(int position, double avg, int block_size)
{
  double num_k = double(block_size * 1024);
  if(avg < MinTime)
    printf("#%d ++++ %f \n", position * block_size, avg);
  else
    printf("%d %d %f\n", position * block_size, int(num_k / avg), avg);
}

int compar(const void *a, const void *b)
{
  double *c = (double *)(a);
  double *d = (double *)(b);
  if(*c < *d) return -1;
  if(*c > *d) return 1;
  return 0;
}

// Returns the mean of the values in the array.  If the array contains
// more than 2 items then discard the highest and lowest thirds of the
// results before calculating the mean.
double average(double *array, int count)
{
  qsort(array, count, sizeof(double), compar);
  int skip = count / 3;
  int arr_items = count - (skip * 2);
  double total = 0.0;
  for(int i = skip; i < (count - skip); i++)
  {
    total += array[i];
  }
  return total / arr_items;
}

// just like the read() system call but takes a (char *) and will not return
// a partial result.
ssize_t readall(int fd, char *buf, size_t count)
{
  ssize_t total = 0;
  while(total != static_cast<ssize_t>(count) )
  {
    ssize_t rc = read(fd, &buf[total], count - total);
    if(rc == -1 || rc == 0)
      return -1;
    total += rc;
  }
  return total;
}

// Read the specified number of megabytes of data from the fd and return the
// amount of time elapsed in seconds.
double readmegs(int fd, int size, char *buf)
{
  struct timeval tp;
 
  if (gettimeofday(&tp, static_cast<struct timezone *>(NULL)) == -1)
  {
    printf("Can't get time.\n");
    return -1.0;
  }
  double start = double(tp.tv_sec) +
    (double(tp.tv_usec) / 1000000.0);

  for(int i = 0; i < size; i++)
  {
    int rc = readall(fd, buf, meg);
    if(rc != meg)
      return -1.0;
  }
  if (gettimeofday(&tp, static_cast<struct timezone *>(NULL)) == -1)
  {
    printf("Can't get time.\n");
    return -1.0;
  }
  return (double(tp.tv_sec) + (double(tp.tv_usec) / 1000000.0))
        - start;
}

