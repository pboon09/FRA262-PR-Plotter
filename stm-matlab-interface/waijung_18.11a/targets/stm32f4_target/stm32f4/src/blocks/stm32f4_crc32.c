#define S_FUNCTION_NAME  stm32f4_crc32
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#define NPAR 9 /* Total number of block parameters */

#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, 0))
#define SAMPLETIMESTR(S) ssGetSFcnParam(S, 1)
#define BLOCKID(S) ssGetSFcnParam(S, 2)
#define POLYDEC(S) (uint32_T) mxGetScalar(ssGetSFcnParam(S, 3))
#define CRCINITDEC(S) (uint32_T) mxGetScalar(ssGetSFcnParam(S, 4))
#define CRCXORDEC(S) (uint32_T) mxGetScalar(ssGetSFcnParam(S, 5))
#define DIRECT(S) mxGetScalar(ssGetSFcnParam(S, 6))
#define REFIN(S) (uint32_T) mxGetScalar(ssGetSFcnParam(S, 7))
#define REFOUT(S) (uint32_T) mxGetScalar(ssGetSFcnParam(S, 8))

static void mdlInitializeSizes(SimStruct *S) {
	int k;
	ssSetNumSFcnParams(S, NPAR);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
	for (k = 0; k < NPAR; k++) {
		ssSetSFcnParamNotTunable(S, k);
	}
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
	
	/* Configure Input Port */
	if (!ssSetNumInputPorts(S, 1)) return; /* Number of input ports */
	
	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDataType(S, 0, SS_UINT32);
	ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
	
	
	/* Configure Output Port */
	if (!ssSetNumOutputPorts(S, 1)) return; /* Number of output ports */
	ssSetOutputPortDataType(S, 0, SS_UINT32);
	ssSetOutputPortWidth(S, 0, 1);
	
	
	ssSetNumSampleTimes(S, 1);
	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S) {
	ssSetSampleTime(S, 0, SAMPLETIME(S));
} /* end mdlInitializeSampleTimes */

const uint32_T order = 32;
//const uint32_T polynom = POLYDEC(S);
//const uint32_T crcinit = CRCINITDEC(S);
//const uint32_T crcxor = CRCXORDEC(S);
//const int direct = DIRECT(S);
//const int refin = REFIN(S);
//const int refout = REFOUT(S);

// internal global values:
uint32_T crcmask;
uint32_T crchighbit;
uint32_T crcinit_direct;
uint32_T crcinit_nondirect;
uint32_T crctab[256];

// CRC parameters (default values are for CRC-32):
//http://www.zorc.breitbandkatze.de/crc.html

// 'order' [1..32] is the CRC polynom order, counted without the leading '1' bit
// 'polynom' is the CRC polynom without leading '1' bit
// 'direct' [0,1] specifies the kind of algorithm: 1=direct, no augmented zero bits
// 'crcinit' is the initial CRC value belonging to that algorithm
// 'crcxor' is the final XOR value
// 'refin' [0,1] specifies if a data byte is reflected before processing (UART) or not
// 'refout' [0,1] specifies if the CRC will be reflected before XOR

// Data character string
// const uint8_T string[] = {"123456789123"};
// const uint8_T string[] = {"87654321"};

// subroutines
uint32_T reflect(uint32_T crc, int bitnum) {
	// reflects the lower 'bitnum' bits of 'crc'
	uint32_T i, j=1, crcout=0;
	for (i=(uint32_T)1<<(bitnum-1); i; i>>=1) {
		if (crc & i) crcout|=j;
		j<<= 1;
	}
	return (crcout);
}

void generate_crc_table(SimStruct *S) {
	// make CRC lookup table used by table algorithms
	int i, j;
	uint32_T bit, crc;
	for (i=0; i<256; i++) {
		crc=(uint32_T)i;
		if (REFIN(S)) crc=reflect(crc, 8);
		crc<<= order-8;
		for (j=0; j<8; j++) {
			bit = crc & crchighbit;
			crc<<= 1;
			if (bit) crc^=POLYDEC(S);
		}
		if (REFIN(S)) crc = reflect(crc, order);
		crc&= crcmask;
		crctab[i]= crc;
	}
}

uint32_T crctablefast(uint8_T* p, uint32_T len, SimStruct *S) {
	// fast lookup table algorithm without augmented zero bytes, e.g. used in pkzip.
	// only usable with polynom orders of 8, 16, 24 or 32.
	uint32_T crc = crcinit_direct;
	if (REFIN(S)) crc = reflect(crc, order);
	if (!REFIN(S)) while (len--) crc = (crc << 8) ^ crctab[ ((crc >> (order-8)) & 0xff) ^ *p++];
	else while (len--) crc = (crc >> 8) ^ crctab[ (crc & 0xff) ^ *p++];
	if (REFOUT(S)^REFIN(S)) crc = reflect(crc, order);
	crc^= CRCXORDEC(S);
	crc&= crcmask;
	return(crc);
}

uint32_T crctable(uint8_T* p, uint32_T len, SimStruct *S) {
	// normal lookup table algorithm with augmented zero bytes.
	// only usable with polynom orders of 8, 16, 24 or 32.
	uint32_T crc = crcinit_nondirect;
	if (REFIN(S)) crc = reflect(crc, order);
	if (!REFIN(S)) while (len--) crc = ((crc << 8) | *p++) ^ crctab[ (crc >> (order-8))  & 0xff];
	else while (len--) crc = ((crc >> 8) | (*p++ << (order-8))) ^ crctab[ crc & 0xff];
	if (!REFIN(S)) while (++len < order/8) crc = (crc << 8) ^ crctab[ (crc >> (order-8))  & 0xff];
	else while (++len < order/8) crc = (crc >> 8) ^ crctab[crc & 0xff];
	if (REFOUT(S)^REFIN(S)) crc = reflect(crc, order);
	crc^= CRCXORDEC(S);
	crc&= crcmask;
	return(crc);
}

uint32_T crcbitbybit(uint8_T* p, uint32_T len, SimStruct *S) {
	
	// bit by bit algorithm with augmented zero bytes.
	// does not use lookup table, suited for polynom orders between 1...32.
	uint32_T i, j, c, bit;
	uint32_T crc = crcinit_nondirect;
	for (i=0; i<len; i++) {
		c = (uint32_T)*p++;
		if (REFIN(S)) c = reflect(c, 8);
		for (j=0x80; j; j>>=1) {
			bit = crc & crchighbit;
			crc<<= 1;
			if (c & j) crc|= 1;
			if (bit) crc^= POLYDEC(S);
		}
	}
	for (i=0; i<order; i++) {
		bit = crc & crchighbit;
		crc<<= 1;
		if (bit) crc^= POLYDEC(S);
	}
	if (REFOUT(S)) crc=reflect(crc, order);
	crc^= CRCXORDEC(S);
	crc&= crcmask;
	return(crc);
}

uint32_T crcbitbybitfast(uint8_T* p, uint32_T len, SimStruct *S) {
	// fast bit by bit algorithm without augmented zero bytes.
	// does not use lookup table, suited for polynom orders between 1...32.
	uint32_T i, j, c, bit;
	uint32_T crc = crcinit_direct;
	for (i=0; i<len; i++) {
		c = (uint32_T)*p++;
		if (REFIN(S)) c = reflect(c, 8);
		for (j=0x80; j; j>>=1) {
			bit = crc & crchighbit;
			crc<<= 1;
			if (c & j) bit^= crchighbit;
			if (bit) crc^= POLYDEC(S);
		}
	}
	if (REFOUT(S)) crc=reflect(crc, order);
	crc^= CRCXORDEC(S);
	crc&= crcmask;
	return(crc);
}

#define MDL_ENABLE
#if defined(MDL_ENABLE) && defined(MATLAB_MEX_FILE)
void mdlEnable(SimStruct *S){
	// at first, compute constant bit masks for whole CRC and CRC high bit
	crcmask = ((((uint32_T)1<<(order-1))-1)<<1)|1;
	crchighbit = (uint32_T)1<<(order-1);
	
	// generate lookup table
	generate_crc_table(S);
}
#endif

#define MDL_DISABLE
#if defined(MDL_DISABLE) && defined(MATLAB_MEX_FILE)
static void mdlDisable(SimStruct *S){
}
#endif


static void mdlOutputs(SimStruct *S, int_T tid) {
	uint32_T bit, crc, i, k, tempuint32;
	uint32_T portwidth = ssGetInputPortWidth(S, 0);
	uint32_T sizebytes = portwidth * 4U;
	uint32_T *y = ssGetOutputPortRealSignal(S, 0);
	InputPtrsType UPtr = ssGetInputPortSignalPtrs(S, 0);
	InputUInt32PtrsType u2 = (InputUInt32PtrsType) UPtr;
	uint8_T *p2 = (uint8_T *) u2[0];
	uint8_T *p8 = (uint8_T *) mxMalloc(sizebytes);
	
	for (i=0; i<portwidth; i++) {
		tempuint32 = *u2[i];
		for (k=0; k<4; k++) {
			p8[(i*4)+k] = (uint8_T) ((tempuint32 >> ((3-k)*8)) & 0xFF);
		}
	}
	
	//uint32([825373492, 892745528])
	
	if (!DIRECT(S)) {
		crcinit_nondirect = CRCINITDEC(S);
		crc = CRCINITDEC(S);
		for (i=0; i<order; i++) {
			bit = crc & crchighbit;
			crc<<= 1;
			if (bit) crc^= POLYDEC(S);
		}
		crc&= crcmask;
		crcinit_direct = crc;
	}
	
	else {
		crcinit_direct = CRCINITDEC(S);
		crc = CRCINITDEC(S);
		for (i=0; i<order; i++) {
			bit = crc & 1;
			if (bit) crc^= POLYDEC(S);
			crc >>= 1;
			if (bit) crc|= crchighbit;
		}
		crcinit_nondirect = crc;
	}
	
	//*y = crcbitbybitfast(p2, sizebytes, S);
	*y = crcbitbybitfast(p8, sizebytes, S);
	
	// call CRC algorithms using the CRC parameters above and print result to the console
	/*
	printf("\n");
	printf("CRC tester v1.1 written on 13/01/2003 by Sven Reifegerste (zorc/reflex)\n");
	printf("-----------------------------------------------------------------------\n");
	printf("\n");
	printf("Parameters:\n");
	printf("\n");
	printf(" port width          :  %d\n", portwidth);
	printf(" polynom             :  0x%X\n", POLYDEC(S));
	printf(" order               :  %d\n", order);
	printf(" crcinit             :  0x%X direct, 0x%X nondirect\n", crcinit_direct, crcinit_nondirect);
	printf(" crcxor              :  0x%X\n", CRCXORDEC(S));
	printf(" refin               :  %d\n", REFIN(S));
	printf(" refout              :  %d\n", REFOUT(S));
	printf("\n");
	printf(" data string         :  '%s' (%d bytes)\n", string, strlen(string));
	printf("\n");
	printf("Results:\n");
	printf("\n");
	printf(" crc bit by bit      :  0x%x\n", crcbitbybit((unsigned char *)string, strlen(string), S));
	printf(" crc bit by bit fast :  0x%x\n", crcbitbybitfast((unsigned char *)string, strlen(string), S));
	printf(" p2 crc bit by bit      :  0x%x\n", crcbitbybit(p2, (uint32_T) portwidth*4U, S));
	printf(" p2 crc bit by bit fast :  0x%x\n", crcbitbybitfast(p2, (uint32_T) portwidth*4U, S));
	 */
	
	mxFree(p8);
} /* end mdlOutputs */

static void mdlTerminate(SimStruct *S) {
	/* do nothing */
} /* end mdlTerminate */

#define MDL_RTW
/* Use mdlRTW to save parameter values to model.rtw file
 * for TLC processing. The name appear withing the quotation
 * "..." is the variable name accessible by TLC.
 */
static void mdlRTW(SimStruct *S) {
	int NOutputPara = 3; /* Number of parameters to output to model.rtw */
	const char * blockid = mxArrayToString(BLOCKID(S));
	const char * sampletimestr = mxArrayToString(SAMPLETIMESTR(S));
	
	if (!ssWriteRTWParamSettings(S, NOutputPara,
			SSWRITE_VALUE_QSTR, "sampletimestr", sampletimestr,
			SSWRITE_VALUE_NUM, "arraysize", (real_T) ssGetInputPortWidth(S, 0),
			SSWRITE_VALUE_QSTR, "blockid", blockid
			)) {
		return; /* An error occurred which will be reported by SL */
	}
	mxFree(sampletimestr);
	mxFree(blockid);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f4_crc32.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f4_crc32.c"
#endif

