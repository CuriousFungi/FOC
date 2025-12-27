/**
 ******************************************************************************
 * @file      sysmem.c
 * @brief     System memory allocation (_sbrk) for newlib
 *
 * This implementation provides the _sbrk() function used by malloc()
 * and friends to grow the heap. It prevents the heap from growing into
 * the reserved main stack region.
 ******************************************************************************
 */

#include <errno.h>
#include <stdint.h>
#include <stddef.h>  /* for ptrdiff_t */

/* -------------------------------------------------------------------------- */
/* Linker symbols                                                             */
/* -------------------------------------------------------------------------- */
/* These are defined in the linker script. We treat their ADDRESSES as the
 * actual RAM boundaries.
 */
extern uint8_t  _end;            /* First address after .data and .bss       */
extern uint8_t  _estack;         /* Top of stack                             */
extern uint32_t _Min_Stack_Size; /* Reserved stack size (in bytes)           */

/* Pointer to the current high watermark of the heap usage */
static uint8_t *__sbrk_heap_end = NULL;

/* -------------------------------------------------------------------------- */
/* _sbrk implementation                                                       */
/* -------------------------------------------------------------------------- */
__attribute__((optimize("O0")))
void *_sbrk(ptrdiff_t incr)
{
  /* Compute the lowest address the stack is allowed to use:
   *   stack_limit = &_estack - _Min_Stack_Size
   */
  const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
  const uint8_t *max_heap    = (const uint8_t *)stack_limit;
  uint8_t       *prev_heap_end;

  /* Initialize heap end at first call */
  if (__sbrk_heap_end == NULL)
  {
    __sbrk_heap_end = &_end;
  }

  /* Protect heap from growing into the reserved MSP stack region */
  if ((__sbrk_heap_end + incr) > max_heap)
  {
    errno = ENOMEM;
    return (void *)-1;
  }

  prev_heap_end    = __sbrk_heap_end;
  __sbrk_heap_end += incr;

  return (void *)prev_heap_end;
}
