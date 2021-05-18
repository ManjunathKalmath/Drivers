//Linux driver Shakti GPIO

#include <stdint.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>


/*!General Purpose Input / Output */
#define GPIO_START 0x00040100 //GPIO Start Address */
#define GPIO_OFFSET 0x08 /*!Generic offset used to access GPIO registers*/
#define GPIO_DIRECTION_CNTRL_REG (uint32_t*) (GPIO_START  + (0 * GPIO_OFFSET ))
#define GPIO_DATA_REG (uint32_t*) (GPIO_START + (1 * GPIO_OFFSET ))
/*
 * General Purpose IOs supported
 */

#define GPIO0  (1 <<  0)
#define GPIO1  (1 <<  1)
#define GPIO2  (1 <<  2)
#define GPIO3  (1 <<  3)
#define GPIO4  (1 <<  4)
#define GPIO5  (1 <<  5)
#define GPIO6  (1 <<  6)
#define GPIO7  (1 <<  7)
#define GPIO8  (1 <<  8)
#define GPIO9  (1 <<  9)
#define GPIO10 (1 << 10)
#define GPIO11 (1 << 11)
#define GPIO12 (1 << 12)
#define GPIO13 (1 << 13)
#define GPIO14 (1 << 14)
#define GPIO15 (1 << 15)
#define GPIO16 (1 << 16)
#define GPIO17 (1 << 17)
#define GPIO18 (1 << 18)
#define GPIO19 (1 << 19)
#define GPIO20 (1 << 20)
#define GPIO21 (1 << 21)
#define GPIO22 (1 << 22)
#define GPIO23 (1 << 23)
#define GPIO24 (1 << 24)
#define GPIO25 (1 << 25)
#define GPIO26 (1 << 26)
#define GPIO27 (1 << 27)
#define GPIO28 (1 << 28)
#define GPIO29 (1 << 29)
#define GPIO30 (1 << 30)
#define GPIO31 (1 << 31)

//some macros
#define ENABLE 	 1	
#define DISABLE  0
#define SET	ENABLE
#define RESET	DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#define GPIO ((GPIO_RegDef_t *)GPIO_START);

typedef struct{
	volatile uint_32t GPIO_DIRECTION_CNTRL_REG;
	volatile uint_32t GPIO_DATA_REG;
}GPIO_RegDef_t;

GPIO_RegDef_t *pGPIO = GPIO;


struct shakti_gpio {
	struct gpio gp;
	struct clk *pclk;
	void __iomem *regs;
	u32 bypass_orig;
};

static int shakti_gpio_remove(struct platform_device *pdev)
{
	struct shakti_gpio *sgpio = platform_get_drvdata(pdev);

	iowrite32(sgpio->bypass_orig, cgpio->regs + CDNS_GPIO_BYPASS_MODE);
	clk_disable_unprepare(sgpio->pclk);

	return 0;
}

static const struct of_device_id cdns_of_ids[] = {
	{ .compatible = "cdns,gpio-r1p02" },
	{ /* sentinel */ },
};

static struct platform_driver cdns_gpio_driver = { //TODO
	.driver = {
		.name = "shakti-gpio",
		.of_match_table = cdns_of_ids,
	},
	.probe = cdns_gpio_probe,
	.remove = cdns_gpio_remove,
};
module_platform_driver(cdns_gpio_driver);

MODULE_AUTHOR("SandLogic Technologies Pvt Ltd");
MODULE_DESCRIPTION("ShaktiVajra GPIO driver");
MODULE_LICENSE("GPL");
