#include "mon.h"


static void* map_base = (void*)(-1);

int setpins(int pin1, int val1, int pin2, int val2, int baseadd)
{
  //Set pin 1, clear pin 2
  //0b10 & ~(0b100)
  //0b000010
  //0b111011
  int pinretval;
  if ((val1 == 1) && (val2 == 1))
    {
      pinretval = _monitor(baseadd, 1, _monitor(baseadd,0,0) | 1<<pin1 | 1<<pin2);
    }
  else if ((val1 ==1) && (val2 ==0))
    {
      pinretval = _monitor(baseadd, 1, (_monitor(baseadd,0,0) | 1<<pin1) & ~(1<<pin2));
    }
  else if ((val1 ==0) && (val2 ==1))
    {
      pinretval = _monitor(baseadd, 1,  (_monitor(baseadd,0,0) | 1<<pin2) & ~(1<<pin1));
    }
  else if ((val1 ==0) && (val2 ==0))
    {
      pinretval = _monitor(baseadd, 1,  (_monitor(baseadd,0,0) & ~(1<<pin1)) & ~(1<<pin2));
    }
  return pinretval;
}



int _monitor(unsigned long the_addr, int write, unsigned long the_value) {
  int fd = -1;
  int retval = EXIT_SUCCESS;

  if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;

  {
    unsigned long addr;
    unsigned long *val = NULL;
    unsigned long incoming_value;
    int access_type = 'w';
    ssize_t val_count = 0;
    //parse_from_argv(argc, argv, &addr, &access_type, &val, &val_count);

    addr = the_addr;
    if (write == 1)
      {
	val_count = 1;
	incoming_value = the_value;
	val = &incoming_value;
      }
    /* Map one page */
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, addr & ~MAP_MASK);
    if(map_base == (void *) -1) FATAL;

    if (addr != 0) {
      if (val_count == 0) {
	//printf(" read_value addr=0x%x\n", (unsigned int)addr);
	retval = read_value(addr);//pass back the value read
      }
      else {
	//printf(" write_values addr=0x%x the_value=0x%x\n", (unsigned int)addr, (unsigned int)the_value);
	write_values(addr, access_type, val, val_count);
      }
    }
    if (map_base != (void*)(-1)) {
      if(munmap(map_base, MAP_SIZE) == -1) FATAL;
      map_base = (void*)(-1);
    }
#if DEBUG_MONITOR
    //printf("addr/type: %lu/%c\n", addr, access_type);

    //printf("val (%ld):", val_count);
    for (ssize_t i = 0; i < val_count; ++i) {
      printf("%lu ", val[i]);
    }

    if (val != NULL) {
      free(val);
      val = NULL;
    }
    printf("\n");
#endif
  }

  if (map_base != (void*)(-1)) {
    if(munmap(map_base, MAP_SIZE) == -1) FATAL;
  }
  if (fd != -1) {
    close(fd);
  }

  return retval;
}

uint32_t read_value(uint32_t a_addr) {
  void* virt_addr = map_base + (a_addr & MAP_MASK);
  uint32_t read_result = 0;
  read_result = *((uint32_t *) virt_addr);
  //printf("0x%08x\n", read_result);
  fflush(stdout);
  return read_result;
}

void write_values(unsigned long a_addr, int a_type, unsigned long* a_values, ssize_t a_len) {
  void* virt_addr = map_base + (a_addr & MAP_MASK);
	
  for (ssize_t i = 0; i < a_len; ++i) {
    switch(a_type) {
    case 'b':
      *((unsigned char *) virt_addr) = a_values[i];
      break;
    case 'h':
      *((unsigned short *) virt_addr) = a_values[i];
      break;
    case 'w':
      *((unsigned long *) virt_addr) = a_values[i];
      break;
    }
  }

  fflush(stdout);
}
