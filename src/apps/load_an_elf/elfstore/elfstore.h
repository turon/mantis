#ifndef __ELFSTORE_H__
#define __ELFSTORE_H__

/**
 * \brief      Read ELF data into buf.
 * \param buf  The buffer to which data should be read.
 * \param offset   Offset of ELF data
 * \param len  The number of bytes that should be read.
 * \return     The number of bytes that was actually read into buf.
 */
int elfstore_read(char *buf, unsigned short offset, unsigned int len);

/**
 * \brief      Write buf to ELF data.
 * \param buf  The buffer from which data should be written.
 * \param offset   Offset of ELF data
 * \param len  The number of bytes that should be written.
 * \return     The number of bytes that was actually written from buf.
 */
int elfstore_write(char *buf, unsigned short offset, unsigned int len);

#endif /* __ELFSTORE_H__ */
