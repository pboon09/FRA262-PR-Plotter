#define S_FUNCTION_NAME  stm32f0_eep_emulation
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#define NPAR __PARAM_COUNT /* Total number of block parameters */

enum {
    ARCG_CONF = 0,    
    
    ARGC_INPUT_PORTTYPE,
    ARGC_INPUT_PORTWIDTH,
    ARGC_OUTPUT_PORTTYPE,
    ARGC_OUTPUT_PORTWIDTH,
    
	ARGC_FILENAME,
	ARGC_INFO,
	ARGC_VARNAME,

    ARGC_OPTIONSTRING,
	
    ARGC_SAMPLETIME,
    ARGC_BLOCKID,
	
    __PARAM_COUNT
};

#define ENABLE_ISR(S) mxGetScalar(ssGetSFcnParam(S, ARGC_ISR_ENABLE))
#define SAMPLETIME(S) mxGetScalar(ssGetSFcnParam(S, ARGC_SAMPLETIME)) /* Sample time (sec) */

static void mdlInitializeSizes(SimStruct *S) {
    int k;
    int input_count;
    int output_count;
    int input_width_count;
    int output_width_count;
	int width;
    
    int priority = 0;
    
    ssSetNumSFcnParams(S, NPAR);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    for (k = 0; k < NPAR; k++) {
        ssSetSFcnParamNotTunable(S, k);
    }
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    
    /* Port count */
    input_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INPUT_PORTTYPE));
    output_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_OUTPUT_PORTTYPE));
    input_width_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_INPUT_PORTWIDTH));
    output_width_count = (int)mxGetNumberOfElements(ssGetSFcnParam(S, ARGC_OUTPUT_PORTWIDTH));
    
    /* Configure Input Port */
    if (!ssSetNumInputPorts(S, input_count)) return; /* Number of input ports */
    
    /* Configure Output Port */
    if (!ssSetNumOutputPorts(S, output_count)) return; /* Number of output ports */
    
    /* Port */
    for(k=0; k<output_count; k++) {		
        if(k<output_width_count) {
			width = (int)(((real_T*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_PORTWIDTH)))[k]);
            ssSetOutputPortWidth(S, k, (width>0)?width:1);
		}
        else {
            ssSetOutputPortWidth(S, k, 1);
		}
        ssSetOutputPortDataType(S, k, (int)(((real_T*)mxGetPr(ssGetSFcnParam(S, ARGC_OUTPUT_PORTTYPE)))[k]));
    }
    
    for(k=0; k<input_count; k++) {		
        ssSetInputPortDirectFeedThrough(S, k, 1);
        if(k<input_width_count) {
			width = (int)(((real_T*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_PORTWIDTH)))[k]);
            ssSetInputPortWidth(S, k, (width>0)?width:1);
		}
        else {
            ssSetInputPortWidth(S, k, 1);
		}
        ssSetInputPortDataType(S, k, (int)(((real_T*)mxGetPr(ssGetSFcnParam(S, ARGC_INPUT_PORTTYPE)))[k]));
		ssSetInputPortRequiredContiguous(S, k, 1); //direct input signal access
    }
    
    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);

} /* end mdlInitializeSizes */

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, SAMPLETIME(S));
} /* end mdlInitializeSampleTimes */

#define MDL_ENABLE
#if defined(MDL_ENABLE) && defined(MATLAB_MEX_FILE)
void mdlEnable(SimStruct *S){
}
#endif

#define MDL_DISABLE
#if defined(MDL_DISABLE) && defined(MATLAB_MEX_FILE)
static void mdlDisable(SimStruct *S){
}
#endif

// ========================================================================
// EEProm data
// ========================================================================
#define EEP_EMULATION_SECTOR_SIZE (2*1024)
#define EEP_EMULATION_SECTOR_COUNT 3
static unsigned char eep_emulation_data[EEP_EMULATION_SECTOR_SIZE * EEP_EMULATION_SECTOR_COUNT];
static unsigned int eep_active_sector = -1;
static unsigned int eep_active_sector_pointer = -1;

#define EEP_EMULATION_MAXDATA 48
static unsigned char eep_emulation_datatype_count = 0; // 0,1 ... 47
static unsigned char eep_emulation_datatype[EEP_EMULATION_MAXDATA];

static real_T eep_emulation_defaultvalue[EEP_EMULATION_MAXDATA];

/* Format sector header
 */
static void eep_format_sector(SimStruct *S, int sector, unsigned int cycle_count) {
	int j;
	
	//mexPrintf("\nFormat sector: %d, count: %u\n", sector, cycle_count);
	
	// Fill 0xFF entire sector
	memset(&eep_emulation_data[sector*EEP_EMULATION_SECTOR_SIZE], 0xFF, EEP_EMULATION_SECTOR_SIZE);
	// Fill data
	eep_emulation_data[sector*EEP_EMULATION_SECTOR_SIZE + 0x00] = 0xAA;
	eep_emulation_data[sector*EEP_EMULATION_SECTOR_SIZE + 0x01] = 0x55;
	eep_emulation_data[sector*EEP_EMULATION_SECTOR_SIZE + 0x02] = (unsigned char)(cycle_count    ); // Erase count LSB_0
	eep_emulation_data[sector*EEP_EMULATION_SECTOR_SIZE + 0x03] = (unsigned char)(cycle_count>>8 ); // Erase count LSB_1
	eep_emulation_data[sector*EEP_EMULATION_SECTOR_SIZE + 0x04] = (unsigned char)(cycle_count>>16); // Erase count LSB_2
	eep_emulation_data[sector*EEP_EMULATION_SECTOR_SIZE + 0x05] = (unsigned char)(cycle_count>>24); // Erase count LSB_3
	eep_emulation_data[sector*EEP_EMULATION_SECTOR_SIZE + 0x06] = 0xFF; // Reserved
	eep_emulation_data[sector*EEP_EMULATION_SECTOR_SIZE + 0x07] = 0xFF; // Reserved
	eep_emulation_data[sector*EEP_EMULATION_SECTOR_SIZE + 0x08] = eep_emulation_datatype_count; // Var count
	for(j=0; j<EEP_EMULATION_MAXDATA; j++) { // Var type
		eep_emulation_data[sector*EEP_EMULATION_SECTOR_SIZE + 0x09+j] = eep_emulation_datatype[j];
	}
}

/* Get data information by name
 * 0 - Sucess,
 * otherwise - Error
 */
static int eep_getinfo_by_name(SimStruct *S, const char *name, unsigned char *index, unsigned char *datatype) {
	unsigned int field_index, varindex, data_type_tmp;
	char *eepinfo, *p;
	int res = -1;
	
	eepinfo = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_INFO));
	
	/* Default as invalid index */
	*index = 0xFF;
	*datatype = 0; /* Assume real_T */
	
	/* Split */
	varindex = 0;
	field_index = 0;
	p = strtok(eepinfo, " ,");
	while((p != NULL) && (res == -1)) {
		switch (field_index & 0x03) {
			case 0: /* Name */
				if(!strcmp(p, name)) {
					/* Update return index value */
					*index = (unsigned char)varindex;
					/* Datatype */
					field_index ++;
					p = strtok(NULL, " ,");
					if((p != NULL) && (sscanf(p, "%u", &data_type_tmp) == 1)) {						
						*datatype = (unsigned char) data_type_tmp;
					}
					/* Result */
					res = 0;
				}
				varindex ++;
				break;
				
			case 1: /* Data type */
			case 2: /* Initial value */
			case 3: /* Reserved */
			default:
				break;
		}
		/* Update field index */
		field_index ++;
		
		/* Search token */
		p = strtok(NULL, " ,");
	}
	
	/* Free allocated memory */
	mxFree(eepinfo);	
	
	// Return operation status
	return res;
}

/* Get data information by index
 * 0 - Sucess,
 * otherwise - Error
 */
static int eep_getinfo_by_index(SimStruct *S, unsigned char index, unsigned char *datatype, real_T *initial_value) {
	unsigned int field_index, varindex, data_type_tmp;
	char *eepinfo, *p;
	int res = -1;
	
	eepinfo = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_INFO));
	
	/* Default as invalid index */
	*datatype = 0; /* Assume real_T */
	
	/* Split */
	varindex = 0;
	field_index = 0;
	p = strtok(eepinfo, " ,");
	while((p != NULL) && (res == -1)) {
		switch (field_index & 0x03) {
			case 0: /* Name */
				break;
				
			case 1: /* Data type */
				if(varindex == index) {
					// Data type
					if (sscanf(p, "%u", &data_type_tmp) == 1) {
						res = 0;
						
						*datatype = (unsigned char)data_type_tmp;
						field_index ++;
						
						p = strtok(NULL, " ,");
						
						*initial_value = (real_T)0; // Default value
						switch(eep_emulation_datatype[varindex]) {
							case 0: // real_T
							case 1: // Single
							{
								real32_T f_val;
								if(sscanf(p, "%f", &f_val) == 1)
									*initial_value = (real_T)f_val;
								break;
							}
							case 2: // Int8
							case 4: // Int16
							case 6: // Int32
							{
								int i_val;
								if(sscanf(p, "%d", &i_val) == 1)
									*initial_value = (real_T)i_val;
								break;
							}							
							case 3: // Uint8
							case 5: // Uint16
							case 7: // Uint32
							{
								unsigned int u_val;
								if(sscanf(p, "%u", &u_val) == 1)
									*initial_value = (real_T)u_val;
								break;
							}
						}
					}
				}
				varindex ++;
				break;
			case 2: /* Initial value */
			case 3: /* Reserved */
			default:
				break;
		}
		/* Update field index */
		field_index ++;
		
		/* Search token */
		p = strtok(NULL, " ,");
	}
	
	/* Free allocated memory */
	mxFree(eepinfo);	
	
	return res;
}

/* Get default (initial) value
 */
static void eep_getdata_defaultvalues(SimStruct *S, real_T *value) {
	unsigned int field_index, varindex;
	char *eepinfo, *p;
	
	eepinfo = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_INFO));
	
	/* Split */
	varindex = 0;
	field_index = 0;
	p = strtok(eepinfo, " ,");
	while((p != NULL) && (varindex < eep_emulation_datatype_count)) {
		switch (field_index & 0x03) {
			case 2: /* Initial value */
				switch(eep_emulation_datatype[varindex]) {
					case 0: // real_T
					case 1: // Single
					{
						real32_T f_val;
						if(sscanf(p, "%f", &f_val) == 1)
							value[varindex] = (real_T)f_val;
						else
							value[varindex] = (real_T)0;
						break;
					}
					case 2: // Int8
					case 4: // Int16
					case 6: // Int32
					{
						int i_val;
						if(sscanf(p, "%d", &i_val) == 1)
							value[varindex] = (real_T)i_val;
						else
							value[varindex] = (real_T)0;
						break;
					}
						
					case 3: // Uint8
					case 5: // Uint16
					case 7: // Uint32
					{
						unsigned int u_val;
						if(sscanf(p, "%u", &u_val) == 1)
							value[varindex] = (real_T)u_val;
						else
							value[varindex] = (real_T)0;
						break;
					}	
				}
				varindex ++;
				break;
				
			case 0: /* Name */				
			case 1: /* Data type */
			case 3: /* Reserved */
			default:
				break;
		}
		/* Update field index */
		field_index ++;
		
		/* Search token */
		p = strtok(NULL, " ,");
	}
	
	/* Free allocated memory */
	mxFree(eepinfo);
}

/* Get value
 */
static int eep_getdata_value(SimStruct *S, const char *name, real_T *output, unsigned char *datatype) {
	unsigned char var_index, var_type, var_key;
	unsigned int search_pos, search_varindex;
	unsigned char *sector;
	//real_T var_value;
	
	// Get var info
	if (eep_getinfo_by_name(S, name, &var_index, &var_type) == 0) {
		*datatype = var_type;
		
		// Search for var value
		search_pos = eep_active_sector_pointer; // Last write data position of active sector
		
		search_varindex = 0xFF;		
		sector = &eep_emulation_data[eep_active_sector*EEP_EMULATION_SECTOR_SIZE];
		var_key = 0xFF;
		
		// Look for specific index
		while ((search_pos >= 64) && (search_varindex != var_index)) {
			// Look for 0xAA
			var_key = 0xFF;
			while ((--search_pos >= 64) && (var_key != 0xAA))
				var_key = sector[search_pos];
			// Look for index
			search_varindex = sector[search_pos];			
		}
		// If found the specific index
		if ((var_key == 0xAA) && (search_varindex == var_index)) {
			switch (var_type) {
				case 0: {// real_T
					real_T tmp_value;
					search_pos -= 8;
					memcpy(&tmp_value, &sector[search_pos], 8);
					*output = tmp_value;
					break;
				}
				case 1: {// single
					real32_T tmp_value;
					search_pos -= 4;
					memcpy(&tmp_value, &sector[search_pos], 4);
					*output = (real_T)tmp_value;					
					break;
				}
				case 2: {// int8
					char tmp_value;
					search_pos -= 2;
					tmp_value = (char)sector[search_pos];
					*output = (real_T)tmp_value;					
					break;
				}
				case 3: {// uint8
					unsigned char tmp_value;
					search_pos -= 2;
					tmp_value = (unsigned char)sector[search_pos];
					*output = (real_T)tmp_value;
					break;
				}
				case 4: {// int16
					short tmp_value;
					search_pos -= 2;
					memcpy(&tmp_value, &sector[search_pos], 2);
					*output = (real_T)tmp_value;
					break;
				}
				case 5: {// uint16
					unsigned short tmp_value;
					search_pos -= 2;
					memcpy(&tmp_value, &sector[search_pos], 2);
					*output = (real_T)tmp_value;
					break;
				}
				case 6: {// int32
					int tmp_value;
					search_pos -= 4;
					memcpy(&tmp_value, &sector[search_pos], 4);
					*output = (real_T)tmp_value;
					break;
				}
				case 7: {// uint32
					unsigned int tmp_value;
					search_pos -= 4;
					memcpy(&tmp_value, &sector[search_pos], 4);
					*output = (real_T)tmp_value;
					break;
				}
				default:
					ssSetErrorStatus(S, "\nEEPROM: Invalid variable type.\n");
					break;
			}
			return 0;
		}
		else {
			return 1; // Data is valid, but never store the value in sector
		}
	}
	else {
		ssSetErrorStatus(S, "\nEEPROM: Invalid variable name.\n");
	}
	
	return -1;
}

// Return 0 on success
static int eep_get_varname(SimStruct *S, unsigned char var_index, char *buffer, int buffer_size) {
	unsigned int field_index, varindex;
	char *eepinfo, *p;
	int res = -1;
	
	eepinfo = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_INFO));
	
	/* Split */
	varindex = 0;
	field_index = 0;
	p = strtok(eepinfo, " ,");
	while((p != NULL) && (res != 0)) {
		switch (field_index & 0x03) {
			case 0: /* Name */
				if (varindex == var_index) {
					// size of name
					if (strlen(p) >= buffer_size) {
						ssSetErrorStatus(S, "\nEEPROM: variable name is too long.\n");
						return res;
					}
					// Copy return value
					strcpy(buffer, p);
					res = 0; // Search complete
				}
				varindex++;
				break;
			case 1: /* Data type */
			case 2: /* Initial value */
			case 3: /* Reserved */
			default:
				break;
		}
		/* Update field index */
		field_index ++;
		
		/* Search token */
		p = strtok(NULL, " ,");
	}	
	/* Free allocated memory */
	mxFree(eepinfo);
	
	return res;
}

static void eep_setdata_value(SimStruct *S, const char *name, real_T value) {
	unsigned char *pBuffer, index, datatype;
	//int bytes_index;
	
	/* Temporary data */
	real_T         d_real_T;
	real32_T          d_single;
	char           d_int8;
	unsigned char  d_uint8;
	short          d_int16;
	unsigned short d_uint16;
	int            d_int32;
	unsigned int   d_uint32;
	
	real_T var_value;
	unsigned char var_type;
	
	if(eep_getinfo_by_name(S, name, &index, &datatype) == 0) {
		/* Active buffer pointer */
		if((eep_active_sector_pointer + 10) >= EEP_EMULATION_SECTOR_SIZE) {
			unsigned int new_sector;
			unsigned int new_sector_pointer;
			unsigned int erase_count;			
			
			// Change active sector
			new_sector = eep_active_sector+1;
			if(new_sector >= EEP_EMULATION_SECTOR_COUNT)
				new_sector = 0;
			/* Get current sector erase count */
			memcpy(&erase_count, &eep_emulation_data[eep_active_sector*EEP_EMULATION_SECTOR_SIZE + 2], 4);
			
			/* Format new sector */
			eep_format_sector(S, new_sector, erase_count+1);
			new_sector_pointer = 64;
			pBuffer = &eep_emulation_data[new_sector*EEP_EMULATION_SECTOR_SIZE];
			
			// Debug
			//mexPrintf("Switch sector: %u, count=%u\n", new_sector, erase_count);
			
			/* Copy existing data to new sector */
			{
				char var_name[256];
				unsigned char var_idx;
				for (var_idx=0; var_idx<eep_emulation_datatype_count; var_idx++) {
					// Get var name
					if (eep_get_varname(S, var_idx, &var_name[0], 256) == 0) {
						// Get data value from specified name
						if( eep_getdata_value(S, (const char *)var_name, &var_value, &var_type) == 0) {
							// Write data to new sector
							// 1. Data
							switch (var_type) {
								case 0:
									d_real_T = var_value;
									memcpy(&pBuffer[new_sector_pointer], &d_real_T, 8);
									new_sector_pointer += 8;
									break;
								case 1:
									d_single = (real32_T)var_value;
									memcpy(&pBuffer[new_sector_pointer], &d_single, 4);
									new_sector_pointer += 4;
									break;
								case 2:
									d_int8 = (char)var_value;
									memcpy(&pBuffer[new_sector_pointer], &d_int8, 1);
									new_sector_pointer += 1;
									memcpy(&pBuffer[new_sector_pointer], &d_int8, 1);
									new_sector_pointer += 1;
									break;
								case 3:
									d_uint8 = (unsigned char)var_value;
									memcpy(&pBuffer[new_sector_pointer], &d_uint8, 1);
									new_sector_pointer += 1;
									memcpy(&pBuffer[new_sector_pointer], &d_uint8, 1);
									new_sector_pointer += 1;
									break;
								case 4:
									d_int16 = (short)var_value;
									memcpy(&pBuffer[new_sector_pointer], &d_int16, 2);
									new_sector_pointer += 2;
									break;
								case 5:
									d_uint16 = (unsigned short)var_value;
									memcpy(&pBuffer[new_sector_pointer], &d_uint16, 2);
									new_sector_pointer += 2;
									break;
								case 6:
									d_int32 = (int)var_value;
									memcpy(&pBuffer[new_sector_pointer], &d_int32, 4);
									new_sector_pointer += 4;
									break;
								case 7:
									d_uint32 = (unsigned int)var_value;
									memcpy(&pBuffer[new_sector_pointer], &d_uint32, 4);
									new_sector_pointer += 4;
									break;
								default:
									// Error, invalid data type
									ssSetErrorStatus(S, "\nEEPROM: Invalid variable type.\n");
									break;
							}
							// 2. index
							pBuffer[new_sector_pointer] = var_idx;
							new_sector_pointer ++;
							// 3. key, 0xAA
							pBuffer[new_sector_pointer] = 0xAA;
							new_sector_pointer ++;
						}
						else {
							// No data storage for at this index
						}
					}
				}
			}
			
			/* Activate new sector */
			{
				unsigned int old_sector;
				
				old_sector = eep_active_sector;
				eep_active_sector = new_sector;
				eep_active_sector_pointer = new_sector_pointer;
				
				/* Clear old sector */
				pBuffer = &eep_emulation_data[old_sector*EEP_EMULATION_SECTOR_SIZE];
				memset(pBuffer, 0xFF,EEP_EMULATION_SECTOR_SIZE);
			}
			//
			//mexPrintf("Active sector: %u, offset: %u\n", eep_active_sector, eep_active_sector_pointer);
		}
		
		/* Get sector buffer */
		pBuffer = &eep_emulation_data[eep_active_sector*EEP_EMULATION_SECTOR_SIZE];
		
		/* Put Data */
		switch (datatype) {
			case 0: // real_T
				// Data
				d_real_T = value;
				memcpy(&pBuffer[eep_active_sector_pointer], &d_real_T, 8);
				eep_active_sector_pointer += 8;
				break;
			case 1: // Single
				// Data
				d_single = (real32_T)value;
				memcpy(&pBuffer[eep_active_sector_pointer], &d_single, 4);
				eep_active_sector_pointer += 4;
				break;
			case 2: // Int8
				// Data
				d_int8 = (char)value;
				memcpy(&pBuffer[eep_active_sector_pointer], &d_int8, 1);
				eep_active_sector_pointer += 1;
				memcpy(&pBuffer[eep_active_sector_pointer], &d_int8, 1);
				eep_active_sector_pointer += 1;
				break;
			case 3: // Uint8
				// Data
				d_uint8 = (unsigned char)value;
				memcpy(&pBuffer[eep_active_sector_pointer], &d_uint8, 1);
				eep_active_sector_pointer += 1;
				memcpy(&pBuffer[eep_active_sector_pointer], &d_uint8, 1);
				eep_active_sector_pointer += 1;
				break;
			case 4: // Int16
				// Data
				d_int16 = (short)value;
				memcpy(&pBuffer[eep_active_sector_pointer], &d_int16, 2);
				eep_active_sector_pointer += 2;
				break;
			case 5: // Uint16
				// Data
				d_uint16 = (unsigned short)value;
				memcpy(&pBuffer[eep_active_sector_pointer], &d_uint16, 2);
				eep_active_sector_pointer += 2;
				break;
			case 6: // Int32
				// Data
				d_int32 = (int)value;
				memcpy(&pBuffer[eep_active_sector_pointer], &d_int32, 4);
				eep_active_sector_pointer += 4;
				break;
			case 7: // Uint32
				// Data
				d_uint32 = (unsigned int)value;
				memcpy(&pBuffer[eep_active_sector_pointer], &d_uint32, 4);
				eep_active_sector_pointer += 4;
				break;
			default:
				ssSetErrorStatus(S, "\nInvalid data type.\n");
		}
		/* Var index */
		pBuffer[eep_active_sector_pointer] = index;
		eep_active_sector_pointer++;
		/*  */
		pBuffer[eep_active_sector_pointer] = 0xAA;
		eep_active_sector_pointer++;
	}
	else {
		ssSetErrorStatus(S, "\nInvalid var name.\n");
	}
}

/* Get data type information
 */
static void eep_getdatatype_info(SimStruct *S, unsigned char *output) {	
	unsigned int field_index, data_type, data_type_count;
	char *eepinfo, *p;
	
	eepinfo = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_INFO));
	
	/* Split */
	field_index = 0;
	data_type_count = 0;
	p = strtok(eepinfo, " ,");
	while(p != NULL) {
		switch (field_index & 0x03) {				
			case 1: /* Data type */
				if (sscanf(p, "%u", &data_type) == 1) {
					output[data_type_count] = (unsigned char)data_type;
					data_type_count ++;
				}
				else { ssSetErrorStatus(S, "\nInvalid data type\n"); }
				break;
				
			case 0: /* Name */				
			case 2: /* Initial value */
			case 3: /* Reserved */
			default:
				break;
		}
		/* Update field index */
		field_index ++;
		
		/* Search token */
		p = strtok(NULL, " ,");
	}
	
	/* Free allocated memory */
	mxFree(eepinfo);
}

/* Load file
 */
static void eep_loadfile(SimStruct *S, unsigned char *output) {
	FILE *f;
	size_t reading_count;
	char *filename;
	
	filename = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_FILENAME));
	f = fopen((const char *)filename, "rb");
	if (f) {
		reading_count = fread(output, 1, (EEP_EMULATION_SECTOR_SIZE*EEP_EMULATION_SECTOR_COUNT), f);
		if(reading_count != (EEP_EMULATION_SECTOR_SIZE*EEP_EMULATION_SECTOR_COUNT)) {
			/* Reset the reading value */
			memset(output, 0xFF, (EEP_EMULATION_SECTOR_SIZE*EEP_EMULATION_SECTOR_COUNT));
		}
			
		/* Close file */
		fclose(f);
	}
	mxFree(filename);
}

/* Save file
 */
static void eep_savefile(SimStruct *S, const unsigned char *input) {
	FILE *f;
	//size_t written_count;
	char *filename;
	
	filename = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_FILENAME));
	f = fopen((const char *)filename, "wb");
	if (f) {
		if(fwrite(input, 1, (EEP_EMULATION_SECTOR_SIZE*EEP_EMULATION_SECTOR_COUNT), f) != \
				(EEP_EMULATION_SECTOR_SIZE*EEP_EMULATION_SECTOR_COUNT)) {
			/* Failed to write into file. */
		}
		/* Close file */
		fclose(f);
	}
	mxFree(filename);	
}

#define MDL_START
static void mdlStart(SimStruct *S) {
	int i;
    char conf[64]; // ARCG_CONF
	char *pconf, *pStr;
	char specific_varname[256]; // Length of var name should limited
	
	unsigned char specific_varindex;
	unsigned char specific_vartype;
	
	/* Get configuration */
    pconf = (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_CONF));
	strcpy(&conf[0], pconf);
	mxFree(pconf);
	/* Get varname */
	if (!strcmp(&conf[0], "Read") || !strcmp(&conf[0], "Write")) {
		pStr = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_VARNAME));
		strcpy(&specific_varname[0], pStr);
		mxFree(pStr);
	}
	else {
		specific_varname[0] = '\0';
	}
	
	if (!strcmp(conf, "Read") || !strcmp(conf, "Write")) {
		int need_reload = 0;
		int active_sector = -1;
		
		//
		//mexPrintf("Startup...\n");
		
		/* Update data types */
		memset(&eep_emulation_datatype[0], 0xFF, sizeof(eep_emulation_datatype));
		eep_getdatatype_info(S, &eep_emulation_datatype[0]);
		eep_emulation_datatype_count = 0;
		for(i=0; i<EEP_EMULATION_MAXDATA; i++) {
			if (eep_emulation_datatype[i] != 0xFF)
				eep_emulation_datatype_count ++;
			else
				break;
		}
		
		/* === Check current working buffer === */
		/* Check if buffer need format */
		for(i=0; i<EEP_EMULATION_SECTOR_COUNT; i++) {
			if ((eep_emulation_data[i*EEP_EMULATION_SECTOR_SIZE] == 0xAA) && (eep_emulation_data[i*EEP_EMULATION_SECTOR_SIZE+1] == 0x55)) {
				active_sector = i;
				break;
			}
		}
		if (active_sector >= 0) {
			/* Sector is valid, then verify its contents */
			if (!((eep_emulation_data[active_sector*EEP_EMULATION_SECTOR_SIZE+8] == eep_emulation_datatype_count) \
					&& !memcmp(&eep_emulation_data[active_sector*EEP_EMULATION_SECTOR_SIZE+9], &eep_emulation_datatype[0], eep_emulation_datatype_count))) {
				active_sector = -1; /* Invalid sector */
			}
		}
		
		/* === Load data from file if buffer is invalid === */
		if (active_sector < 0) {
			//mexPrintf("\n=== Load from file ===\n");
			
			/* Load file, format if file doesn't existed */
			eep_loadfile(S, &eep_emulation_data[0]);
			
			/* Verify data */
			for(i=0; i<EEP_EMULATION_SECTOR_COUNT; i++) {
				if ((eep_emulation_data[i*EEP_EMULATION_SECTOR_SIZE] == 0xAA) && (eep_emulation_data[i*EEP_EMULATION_SECTOR_SIZE+1] == 0x55)) {
					active_sector = i;
					break;
				}
			}
			if (active_sector >= 0) {
				/* Sector is valid, then verify its contents */
				if (!((eep_emulation_data[active_sector*EEP_EMULATION_SECTOR_SIZE+8] == eep_emulation_datatype_count) \
						&& !memcmp(&eep_emulation_data[active_sector*EEP_EMULATION_SECTOR_SIZE+9], &eep_emulation_datatype[0], eep_emulation_datatype_count))) {
					active_sector = -1; /* Invalid sector */
				}
			}			
		}
		
		/* === Reset default === */
		if (active_sector < 0) {
			//int j;
			
			// EEProm file is invalid
			//mexPrintf("\n=== Invalid EEProm ===\n");
			
			/* Default active sector */
			eep_active_sector = 0;
			eep_active_sector_pointer = 64; /* Start */
			
			/* Generating EEPROM file.. */
			/* Format */
			memset(&eep_emulation_data[0], 0xFF, (EEP_EMULATION_SECTOR_SIZE*EEP_EMULATION_SECTOR_COUNT));
			eep_format_sector(S, 0, 0);
		}
		else {
			unsigned char *pBuffer;
			int idx;
			
			/* Current active sector */
			eep_active_sector = active_sector;			
			
			/* Active sector pointer */
			pBuffer = &eep_emulation_data[eep_active_sector*EEP_EMULATION_SECTOR_SIZE];
			idx = EEP_EMULATION_SECTOR_SIZE-1;
			while(idx-- > 64) {
				if (pBuffer[idx] != 0xFF)
					break;					
			}
			eep_active_sector_pointer = (unsigned int)idx;
		}
		
		// Default data value
		{
			int idx;
			for (idx=0; idx<EEP_EMULATION_MAXDATA; idx++)
				eep_emulation_defaultvalue[idx] = 0;
			eep_getdata_defaultvalues(S, &eep_emulation_defaultvalue[0]);
		}
		
		/* Variable Read/Write setting */
		if(eep_getinfo_by_name(S, (const char *)&specific_varname[0], &specific_varindex, &specific_vartype) == 0) {
			// Variable name is valid
			//mexPrintf("Var \"%s\", index=%u, type=%u\n", specific_varname, (unsigned int)specific_varindex, (unsigned int)specific_vartype);
		}
		else { // Error
			mexPrintf("Var \"%s\" is invalid or not defined.\n", specific_varname);
			ssSetErrorStatus(S, "\nUndefined variable.\n");
		}		
		
		// Display sector information
		//mexPrintf("\nActive sector: %u, offset: %u\n", eep_active_sector, eep_active_sector_pointer);
	}
}

/* Output
 */
static void mdlOutputs(SimStruct *S, int_T tid) {
	char *conf;
	char *name;
	real_T value;
	unsigned char datatype, varindex;
	
	// configuration
	conf = (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_CONF));
	name = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_VARNAME));
	
	//
	if (!strcmp(conf, "Read") || !strcmp(conf, "Write")) {
		if(eep_getinfo_by_name(S, name, &varindex, &datatype) != 0) {
			ssSetErrorStatus(S, "\nInvalid var name.\n");
		}
	}
	
	// EEProm read
	if (!strcmp(conf, "Read")) {
		int sta;
		
		sta = eep_getdata_value(S, name, &value, &datatype);
		if (sta == 0) {
			// Success
		}
		else if(sta == 1) {
			// No error, but no storage
			value = eep_emulation_defaultvalue[varindex];
			//mexPrintf("Default value: ");
		}
		else {
			// Error
			ssSetErrorStatus(S, "\nFailed to read EEProm value.\n");
		}
		//mexPrintf("Reading: %f\n", (real32_T)value);
		
		switch((unsigned char)ssGetOutputPortDataType(S, 0)) {
			case 0:
				*(real_T*) ssGetOutputPortSignal(S, 0) = value;
				break;			
			case 1:
				*(real32_T*) ssGetOutputPortSignal(S, 0) = (real32_T)value;
				break;			
			case 2:
				*(int8_T*) ssGetOutputPortSignal(S, 0) = (int8_T)value;
				break;			
			case 3:
				*(uint8_T*) ssGetOutputPortSignal(S, 0) = (uint8_T)value;
				break;			
			case 4:
				*(int16_T*) ssGetOutputPortSignal(S, 0) = (int16_T)value;
				break;			
			case 5:
				*(uint16_T*) ssGetOutputPortSignal(S, 0) = (uint16_T)value;
				break;			
			case 6:
				*(int32_T*) ssGetOutputPortSignal(S, 0) = (int32_T)value;
				break;			
			case 7:
				*(uint32_T*) ssGetOutputPortSignal(S, 0) = (uint32_T)value;
				break;			
			default:
				ssSetErrorStatus(S, "\nPort data type is not support.\n");
				break;
		}
	}
	// EEProm write
	else if (!strcmp(conf, "Write")) {
		switch((unsigned char)ssGetInputPortDataType(S, 0)) {
			case 0:
				value = (real_T)*((const real_T*) ssGetInputPortSignal(S, 0));
				break;			
			case 1: 
				value = (real_T)*((const real32_T*) ssGetInputPortSignal(S, 0));
				break;			
			case 2: 
				value = (real_T)*((const int8_T*) ssGetInputPortSignal(S, 0));
				break;			
			case 3:
				value = (real_T)*((const uint8_T*) ssGetInputPortSignal(S, 0));
				break;
			case 4:
				value = (real_T)*((const int16_T*) ssGetInputPortSignal(S, 0));
				break;
			case 5:
				value = (real_T)*((const uint16_T*) ssGetInputPortSignal(S, 0));
				break;
			case 6:
				value = (real_T)*((const int32_T*) ssGetInputPortSignal(S, 0));
				break;
			case 7:
				value = (real_T)(*((const uint32_T*) ssGetInputPortSignal(S, 0)));
				break;			
			default:
				ssSetErrorStatus(S, "\nPort data type is not support.\n");
				break;
		}
		/* Write data into memory */
		//mexPrintf("Write: %f\n", value);
		eep_setdata_value(S, name, value);
	}
	// EEProm setup
	else {
		/* Setup */
	}
	
	// Free
	mxFree(conf);
	mxFree(name);
} /* end mdlOutputs */

static void mdlTerminate(SimStruct *S) {
	int i, active_sector;
	
	/* Check if there are active sector on buffer */
	active_sector = -1;
	for(i=0; i<EEP_EMULATION_SECTOR_COUNT; i++) {
		if ((eep_emulation_data[i*EEP_EMULATION_SECTOR_SIZE] == 0xAA) && (eep_emulation_data[i*EEP_EMULATION_SECTOR_SIZE+1] == 0x55)) {
			active_sector = i;
			break;
		}
	}
	
	/* Save to file if current working buffer is valid */
	if (active_sector >= 0) {
		/* Save to file*/
		eep_savefile(S, &eep_emulation_data[0]);
		/* Clear */
		memset(&eep_emulation_data[0], 0xFF, (EEP_EMULATION_SECTOR_SIZE*EEP_EMULATION_SECTOR_COUNT));
	}
} /* end mdlTerminate */

static int get_list_count(char *s)
{
    char *p;
    int count;
    
    count = 1;
    p = s;
    while((p=strstr(p, ",")) != (void*)0) {
        count ++;
        p++;
    }
    return count;
}

#define MDL_RTW
/* Use mdlRTW to save parameter values to model.rtw file
 * for TLC processing. The name appear withing the quotation
 * "..." is the variable name accessible by TLC.
 */
static void mdlRTW(SimStruct *S) {
    int NOutputPara = 3; /* Number of parameters to output to model.rtw */
    
    char *conf; // ARCG_CONF
    char* optionstring;
    char *sampletimestr;
    char *blockid;
    
    conf = (char*)mxArrayToString(ssGetSFcnParam(S, ARCG_CONF));
    optionstring = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_OPTIONSTRING));    
    blockid = (char*)mxArrayToString(ssGetSFcnParam(S, ARGC_BLOCKID));
    
    if (!ssWriteRTWParamSettings(S, NOutputPara,
            SSWRITE_VALUE_QSTR, "conf", conf,
            SSWRITE_VALUE_NUM, "sampletime", SAMPLETIME(S),
            SSWRITE_VALUE_QSTR, "blockid", blockid            
            )) {
        return; /* An error occurred which will be reported by SL */
    }

    /* Write configuration string */
    if (!ssWriteRTWStrVectParam(S, "optionstring", optionstring, get_list_count(optionstring))){
        return;
    }    
        
    mxFree(conf);
    mxFree(optionstring);    
    mxFree(blockid);
}

/* Enforce use of inlined S-function      *
 * (e.g. must have TLC file stm32f0_eep_emulation.tlc)  *
 *=======================================*/
#ifdef    MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file?    */
#include "simulink.c"     /* MEX-file interface mechanism                  */
#else                      /* Prevent usage by RTW if TLC file is not found */
#error "Attempted use non-inlined S-function stm32f0_eep_emulation.c"
#endif

