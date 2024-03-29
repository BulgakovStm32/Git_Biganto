/*
 * metadata.h
 *
 *  Created on: 28 мар. 2023 г.
 *      Author: belyaev
 */

#ifndef FOR_BOOTLOADER_METADATA_H_
#define FOR_BOOTLOADER_METADATA_H_
//*******************************************************************************************
//*******************************************************************************************

#include "stdint.h"

//*******************************************************************************************
//*******************************************************************************************
//Метаданные приложения.
//Некоторые матаданные прописываются константами в приложении. К таким константам относятся: версия методанных,
//"волшебное" число, тип приложения, и адрес векторной таблицы. Другие поля должны быть прочитаны из среды
//и введены, как DEFINES в наш Makefile.
//Некоторая информация в для метаданных может быть рассчитана только после того, как прошивка будет
//скомпилирована и слинкована. Например, длина двоичного файла, его CRC или его криптографическая подпись.
//Чтобы добавить его в шапку, мы должны отредактировать бинарник напрямую какой то внешней утилитой.

#define METADATA_SIZE	(1 * 1024)	//размер метаданных приложения, в байтах
#define METADATA_MAGIC	0xcafe		//"магическое" число
//************************************************************
typedef enum{
    APP_TYPE_LOADER 	= 0x1,
	APP_TYPE_APP 		= 0x2,
	APP_TYPE_UPDATER	= 0x3,
}app_type_t;
//************************************************************
//Структура метаданных приложения
typedef struct __attribute__((packed)){

	uint16_t 	metadataVersion;	//версия методанных

	//"магическое" число обычно представляет собой константу, которая используется
	//для идентификации формата файла или структуры данных.
	//В этом случае каждое изображение должно иметь байты 0xcafe в первых двух байтах.
	//Если число отличается - образ недействителен!
	uint16_t 	appMagic;			//"магическое" число

	uint8_t 	appType;			//???
	uint32_t 	appVectorAddr;		//стартовый адрес приложения
	uint32_t 	appSize;			//размер bin-файла приложения в байтах
	uint32_t 	appCrc;				//контрольная сумма bin-файла приложения
	uint8_t 	appVersion_major;	//
	uint8_t 	appVersion_minor;	//
	uint8_t 	appVersion_patch;	//
	char 		git_sha[8];			//хеш гита
	uint32_t 	reserved;			//
}appMetadata_t;
//*******************************************************************************************
//*******************************************************************************************


//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
//*******************************************************************************************
#endif /* FOR_BOOTLOADER_METADATA_H_ */














