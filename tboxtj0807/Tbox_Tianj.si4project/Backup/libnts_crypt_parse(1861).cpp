#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include "libnts_crypt_parse.h"




int encrypt_key()
{
	void *handle;
	callback_encrypt_key cb_encrypt_key = NULL;

	handle = dlopen(LIB_NTS_PATH, RTLD_LAZY);
	if(handle == NULL)
	{
		printf("error:%s \n", dlerror());
		return -1;
	}
	
	cb_encrypt_key = (callback_encrypt_key)dlsym(handle, "nts_encrypt_key");
	if(cb_encrypt_key == NULL)
	{
		printf("dlsym error:%s \n", dlerror());
		return -1;
	}

    printf("open ok!!!!!!!!!!!!!!!! \n");
	/**
	 * use cb_encrypt_key function to call nts_encrypt_key;
	 * need user to achieve this functionality
	 */
	
	
	dlclose(handle);
	
	return 0;
}

int decrypt_key()
{
	void *handle;
	callback_decrypt_key cb_decrypt_key = NULL;
	
	handle = dlopen(LIB_NTS_PATH, RTLD_LAZY);
	if(handle == NULL)
	{
		printf("error:%s \n", dlerror());
		return -1;
	}
	
	cb_decrypt_key = (callback_decrypt_key)dlsym(handle, "nts_decrypt_key");
	if(cb_decrypt_key == NULL)
	{
		printf("dlsym error:%s \n", dlerror());
		return -1;
	}
	
	/**
	 * use cb_decrypt_key function to call nts_decrypt_key;
	 * need user to achieve this functionality
	 */
	
	
	dlclose(handle);

	return 0;
}

