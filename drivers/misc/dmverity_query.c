#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/highmem.h>

#define	CMD_READ_SYSTEM_IMAGE_CHECK_STATUS 3
#ifdef CONFIG_BLOWFISH
#define SMC_NO(x)		"" # x
#define SMC(x)			"smc " SMC_NO(x)

#define CUSTOM_SMC_FID		0xB2000202
#define SUBFUN_DMVERITY_RECOVERY 130

static inline u32 smc_call(u32 cmd, u32 arg1, u32 arg2, u32 arg3)
{
    register u32 _a0 __asm__("x0") = cmd;
    register u32 _a1 __asm__("x1") = arg1;
    register u32 _a2 __asm__("x2") = arg2;
    register u32 _a3 __asm__("x3") = arg3;
    register u32 _r0 __asm__("x0");
    register u32 _r1 __asm__("x1");
    register u32 _r2 __asm__("x2");
    register u32 _r3 __asm__("x3");
			
        __asm__ __volatile__(SMC(0): "=r"(_r0), "=r"(_r1), "=r"(_r2), "=r"(_r3) : "0"(_a0), "r"(_a1), "r"(_a2), "r"(_a3) : "memory");

	arg3 = _r3;
	arg2 = _r2;
	arg1 = _r1;
	cmd  = _r0;
	
	return _r0;
}

#endif

static inline u64 exynos_smc_verity(u64 cmd, u64 arg1, u64 arg2, u64 arg3)
{
    register u64 reg0 __asm__("x0") = cmd;
    register u64 reg1 __asm__("x1") = arg1;
    register u64 reg2 __asm__("x2") = arg2;
    register u64 reg3 __asm__("x3") = arg3;

    __asm__ volatile (
        "dsb    sy\n"
        "smc    0\n"
        : "+r"(reg0), "+r"(reg1), "+r"(reg2), "+r"(reg3)

    );  

    return reg0;
}


static int verity_scm_call(void)
{
#ifdef CONFIG_BLOWFISH	
	return smc_call(CUSTOM_SMC_FID, SUBFUN_DMVERITY_RECOVERY, CMD_READ_SYSTEM_IMAGE_CHECK_STATUS, 0);
#else	
	return exynos_smc_verity(0x83000006, CMD_READ_SYSTEM_IMAGE_CHECK_STATUS, 0, 0);
#endif
}

#define DRIVER_DESC   "Read whether odin flash succeeded"

ssize_t	dmverity_read(struct file *filep, char __user *buf, size_t size, loff_t *offset)
{
	uint32_t	odin_flag;
	//int ret;

	/* First check is to get rid of integer overflow exploits */
	if (size < sizeof(uint32_t)) {
		printk(KERN_ERR"Size must be atleast %d\n", (int)sizeof(uint32_t));
		return -EINVAL;
	}

	odin_flag = verity_scm_call();
	printk(KERN_INFO"dmverity: odin flag: %x\n", odin_flag);

	if (copy_to_user(buf, &odin_flag, sizeof(uint32_t))) {
		printk(KERN_ERR"Copy to user failed\n");
		return -1;
	} else
		return sizeof(uint32_t);
}

static const struct file_operations dmverity_proc_fops = {
	.read		= dmverity_read,
};

/**
 *      dmverity_odin_flag_read_init -  Initialization function for DMVERITY
 *
 *      It creates and initializes dmverity proc entry with initialized read handler 
 */
static int __init dmverity_odin_flag_read_init(void)
{
	//extern int boot_mode_recovery;
	if (/* boot_mode_recovery == */ 1) {
		/* Only create this in recovery mode. Not sure why I am doing this */
        	if (proc_create("dmverity_odin_flag", 0644,NULL, &dmverity_proc_fops) == NULL) {
			printk(KERN_ERR"dmverity_odin_flag_read_init: Error creating proc entry\n");
			goto error_return;
		}
	        printk(KERN_INFO"dmverity_odin_flag_read_init:: Registering /proc/dmverity_odin_flag Interface \n");
	} else {
		printk(KERN_INFO"dmverity_odin_flag_read_init:: not enabling in non-recovery mode\n");
		goto error_return;
	}

        return 0;

error_return:
	return -1;
}


/**
 *      dmverity_odin_flag_read_exit -  Cleanup Code for DMVERITY
 *
 *      It removes /proc/dmverity proc entry and does the required cleanup operations 
 */
static void __exit dmverity_odin_flag_read_exit(void)
{
        remove_proc_entry("dmverity_odin_flag", NULL);
        printk(KERN_INFO"Deregistering /proc/dmverity_odin_flag interface\n");
}


module_init(dmverity_odin_flag_read_init);
module_exit(dmverity_odin_flag_read_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
