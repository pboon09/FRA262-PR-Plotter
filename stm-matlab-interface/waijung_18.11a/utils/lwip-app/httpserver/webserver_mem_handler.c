
#include <stdint.h>
#include <string.h>
#include "webserver_debug_port.h"
#include "webserver_mem_handler.h"

/* !!! Temporary use for for memory pool function. Improve later.
*/

/* Bulk memory */
#if defined   (__CC_ARM) /*!< ARM Compiler */
	__align(4) 
	uint8_t webserver_pool[WEBSERVER_POOL_SIZE];
#elif defined ( __ICCARM__ ) /*!< IAR Compiler */
	#pragma data_alignment=4
	uint8_t webserver_pool[WEBSERVER_POOL_SIZE];
#elif defined (__GNUC__) /*!< GNU Compiler */
	uint8_t webserver_pool[WEBSERVER_POOL_SIZE] __attribute__ ((aligned (4)));
#elif defined  (__TASKING__) /*!< TASKING Compiler */
	__align(4) 
	uint8_t webserver_pool[WEBSERVER_POOL_SIZE];
#else
	#error "Unsupport Compiler!"
#endif

/* Mem status */
static uint8_t webserver_mem_ptr8[WEBSERVER_POOL_SIZE/(WEBSERVER_MEM_PIECE*8)];
static uint16_t *webserver_mem_ptr16 = (uint16_t*)&webserver_mem_ptr8[0];
static uint32_t *webserver_mem_ptr32 = (uint32_t*)&webserver_mem_ptr8[0];

void webserver_mem_init(void) {
	memset(webserver_mem_ptr8, 0, WEBSERVER_POOL_SIZE/(WEBSERVER_MEM_PIECE*8));
}

void *_memtiny_alloc(void) {
	uint16_t i, j;
	uint8_t *mem = 0;
	for(i=0; i<(WEBSERVER_POOL_SIZE/(WEBSERVER_MEM_PIECE*8)); i++) {
		if(webserver_mem_ptr8[i] != 0xFF) {
			for(j=0; j<8; j++) {
				if((webserver_mem_ptr8[i] & ((uint8_t)1<<j)) == 0) { /* Free */
					webserver_mem_ptr8[i] |= (uint8_t)1<<j; /* Acquired */
					mem = (uint8_t*)&webserver_pool[(i*8+j)*WEBSERVER_MEM_PIECE];							
					WEBSERVER_DEBUG_PRINT("Allocate tiny mem, offset: %8X", (uint32_t)(mem-webserver_pool));
					return (void*)mem;
				}
			}
		}
	}
	WEBSERVER_DEBUG_PRINT("Failed to allocate tiny memory");	
	return (void*)0;
}

void *_memtiny_free(void* mem) {
	uint32_t offset;
	if(mem) {
		/* Validate */
		if(((uint32_t)mem < (uint32_t)webserver_pool) \
			|| ((uint32_t)mem > ((uint32_t)webserver_pool+WEBSERVER_POOL_SIZE-WEBSERVER_MEM_PIECE))){
			WEBSERVER_DEBUG_PRINT("_memtiny_free() failed, mem addr: %u", (uint32_t)mem);
			return mem;
		}
		/* free */
		WEBSERVER_DEBUG_PRINT("Free tiny mem, offset: %8X", (uint32_t)((uint8_t*)mem-webserver_pool));
		offset = ((uint32_t)mem - (uint32_t)webserver_pool)/WEBSERVER_MEM_PIECE;
		webserver_mem_ptr8[(offset >> 3)] &= (uint8_t)(~(1<< (offset & 3)));
	}

	return (void*)0;
}

void *_memsmall_alloc(void) {
	uint16_t i;
	uint8_t *mem = 0;
	for(i=0; i<(WEBSERVER_POOL_SIZE/(WEBSERVER_MEM_PIECE*8)); i++) {
		if(webserver_mem_ptr8[i] == 0x00) { /* Free */
			webserver_mem_ptr8[i] = 0xFF; /* Acquired */
			mem = (uint8_t*)&webserver_pool[(i*8)*WEBSERVER_MEM_PIECE];
			WEBSERVER_DEBUG_PRINT("Allocate Small mem, offset: %8X", (uint32_t)(mem-webserver_pool));
			return (void*)mem;					
		}
	}	
	WEBSERVER_DEBUG_PRINT("Failed to allocate small memory");	
	return (void*)0;
}

void *_memsmall_free(void* mem) {
	uint32_t offset;
	if(mem) {
		/* Validate */
		if(((uint32_t)mem < (uint32_t)webserver_pool) \
			|| ((uint32_t)mem > ((uint32_t)webserver_pool+WEBSERVER_POOL_SIZE-(WEBSERVER_MEM_PIECE*8)))){
			WEBSERVER_DEBUG_PRINT("_memsmall_free() failed, mem addr: %u", (uint32_t)mem);
			return mem;
		}
		/* free */
		offset = ((uint32_t)mem - (uint32_t)webserver_pool)/(WEBSERVER_MEM_PIECE);
		if((offset & 0x07) != 0) {
			WEBSERVER_DEBUG_PRINT("_memsmall_free() failed, mem addr: %u", (uint32_t)mem);
			return mem;
		}
		WEBSERVER_DEBUG_PRINT("Free small mem, offset: %8X", (uint32_t)((uint8_t*)mem-webserver_pool));
		webserver_mem_ptr8[offset>>3] = 0x00;
	}

	return (void*)0;
}

void *_memmedium_alloc(void) {
	uint16_t i;
	uint8_t *mem = 0;
	for(i=0; i<(WEBSERVER_POOL_SIZE/(WEBSERVER_MEM_PIECE*16)); i++) {
		if(webserver_mem_ptr16[i] == 0x0000) { /* Free */
			webserver_mem_ptr16[i] = 0xFFFF; /* Acquired */
			mem = (uint8_t*)&webserver_pool[(i*16)*WEBSERVER_MEM_PIECE];
			WEBSERVER_DEBUG_PRINT("Allocate Medium mem, offset: %8X", (uint32_t)(mem-webserver_pool));
			return (void*)mem;					
		}
	}
	/* TODO: Try allocate memory from HEAP	
	*/
	WEBSERVER_DEBUG_PRINT("Failed to allocate medium memory");	
	return (void*)0;
}

void *_memmedium_free(void* mem) {
	uint32_t offset;
	if(mem) {

		/* Validate */
		if(((uint32_t)mem < (uint32_t)webserver_pool) \
			|| ((uint32_t)mem > ((uint32_t)webserver_pool+WEBSERVER_POOL_SIZE-(WEBSERVER_MEM_PIECE*16)))){
			WEBSERVER_DEBUG_PRINT("_memmedium_free() failed, mem addr: %u", (uint32_t)mem);
			return mem;
		}
		/* free */
		offset = ((uint32_t)mem - (uint32_t)webserver_pool)/(WEBSERVER_MEM_PIECE);
		if((offset & 0x0F) != 0) {
			WEBSERVER_DEBUG_PRINT("_memmedium_free() failed, mem addr: %u", (uint32_t)mem);
			return mem;
		}
		WEBSERVER_DEBUG_PRINT("Free medium mem, offset: %8X", (uint32_t)((uint8_t*)mem-webserver_pool));
		webserver_mem_ptr16[offset>>4] = 0x0000;
	}

	return (void*)0;
}

void *_memlarge_alloc(void) {
	uint16_t i;
	uint8_t *mem = 0;
	for(i=0; i<(WEBSERVER_POOL_SIZE/(WEBSERVER_MEM_PIECE*32)); i++) {
		if(webserver_mem_ptr32[i] == 0x00000000) { /* Free */
			webserver_mem_ptr32[i] = 0xFFFFFFFF; /* Acquired */
			mem = (uint8_t*)&webserver_pool[(i*32)*WEBSERVER_MEM_PIECE];
			WEBSERVER_DEBUG_PRINT("Allocate Large mem, offset: %8X", ((uint32_t)(mem-webserver_pool)));
			return (void*)mem;					
		}
	}
	/* TODO: Try allocate memory from HEAP	
	*/
	WEBSERVER_DEBUG_PRINT("Failed to allocate large memory");	
	return (void*)0;
}

void *_memlarge_free(void* mem) {
	uint32_t offset;
	if(mem) {
		/* Validate */
		if(((uint32_t)mem < (uint32_t)webserver_pool) \
			|| ((uint32_t)mem > ((uint32_t)webserver_pool+WEBSERVER_POOL_SIZE-(WEBSERVER_MEM_PIECE*32)))){
			WEBSERVER_DEBUG_PRINT("_memmedium_free() failed, mem addr: %u", (uint32_t)mem);
			return mem;
		}
		/* free */
		offset = ((uint32_t)mem - (uint32_t)webserver_pool)/(WEBSERVER_MEM_PIECE);
		if((offset & 0x1F) != 0) {
			WEBSERVER_DEBUG_PRINT("_memmedium_free() failed, mem addr: %u", (uint32_t)mem);
			return mem;
		}
		WEBSERVER_DEBUG_PRINT("Free large mem, offset: %8X", ((uint32_t)((uint8_t*)mem-webserver_pool)));
		webserver_mem_ptr32[offset>>5] = 0x00000000;
	}

	return (void*)0;
}
