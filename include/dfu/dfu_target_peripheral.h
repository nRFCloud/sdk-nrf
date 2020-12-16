#ifndef DFU_TARGET_PERIPHERAL_H_
#define DFU_TARGET_PERIPHERAL_H_

#include "dfu/dfu_target.h"

#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Initialize the resources needed for the specific image type DFU
 *	  target.
 *
 *	  If a target update is in progress, and the same target is
 *	  given as input, then calling the 'init()' function of that target is
 *	  skipped.
 *
 *	  To allow continuation of an aborted DFU procedure, call the
 *	  'dfu_target_offset_get' function after invoking this function.
 *
 * @param[in] img_type Image type identifier.
 * @param[in] file_size Size of the current file being downloaded.
 * @param[in] cb Callback function in case the DFU operation requires additional
 *		 proceedures to be called.
 *
 * @return 0 for a supported image type or a negative error
 *	   code identicating reason of failure.
 *
 **/
int dfu_target_peripheral_init(size_t file_size, dfu_target_callback_t cb);

/**
 * @brief Get offset of the firmware upgrade
 *
 * @param[out] offset Returns the offset of the firmware upgrade.
 *
 * @return 0 if success, otherwise negative value if unable to get the offset
 */
int dfu_target_peripheral_offset_get(size_t *offset);

/**
 * @brief Write the given buffer to the initialized DFU target.
 *
 * @param[in] buf A buffer of bytes which contains part of an binary firmware
 *		  image.
 * @param[in] len The length of the provided buffer.
 *
 * @return Positive identifier for a supported image type or a negative error
 *	   code identicating reason of failure.
 **/
int dfu_target_peripheral_write(const void *const buf, size_t len);

/**
 * @brief Deinitialize the resources that were needed for the current DFU
 *	  target.
 *
 * @param[in] successful Indicate whether the process completed successfully or
 *			 was aborted.
 *
 * @return 0 for an successful deinitialization or a negative error
 *	   code identicating reason of failure.
 **/
int dfu_target_peripheral_done(bool successful);

#endif /* DFU_TARGET_PERIPHERAL_H_ */
