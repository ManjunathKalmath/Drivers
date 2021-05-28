#include <linux/clk.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/completion.h>
#include <linux/log2.h>
#include <linux/mod_devicetable.h>

#define SHAKTI_SPI_NAME "shakti_spi"
#define SHAKTI_SPI_DEFAULT_DEPTH 	4	//in bytes
#define SHAKTI_SPI_DEFAULT_BITS  	8

#define SPI_CR1	     0x00020000
#define SPI_CR2	     0x00020004
#define SPI_SR       0x00020008
#define SPI_DR1	     0x0002000C
#define SPI_DR2	     0x00020010
#define SPI_DR3	     0x00020014
#define SPI_DR4	     0x00020018
#define SPI_DR5      0x0002001C
#define SPI_CRCPR    0x00020020
#define SPI_RXCRCR   0x00020024
#define SPI_TXCRCR   0x00020028

// defining SPI_CR1 register
#define SPI_CPHA	      (1 << 0)	
#define SPI_CPOL	      (1 << 1)  
#define SPI_MSTR	      (1 << 2)
#define SPI_BR(x)	      (x << 3)	
#define SPI_SPE		      (1 << 6)  
#define SPI_LSBFIRST	      (1 << 7)
#define SPI_SSI		      (1 << 8)
#define SPI_SSM		      (1 << 9)
#define SPI_RXONLY	      (1 << 10)
#define SPI_CRCL	      (1 << 11)
#define SPI_CCRCNEXT	      (1 << 12)
#define SPI_CRCEN	      (1 << 13)
#define SPI_BIDIOE	      (1 << 14)
#define SPI_BIDIMODE	      (1 << 15)
#define SPI_TOTAL_BITS_TX(x)  (x << 16)	
#define SPI_TOTAL_BITS_RX(x)  (x << 24) 

// defining SPI_CR2 register
#define SPI_RX_IMM_START   (1 << 16)
#define SPI_RX_START	   (1 << 15)
#define SPI_LDMA_TX	   (1 << 14)
#define SPI_LDMA_RX	   (1 << 13)
#define SPI_FRXTH	   (1 << 12)
#define SPI_DS(x)	   (x << 8) 
#define SPI_TXEIE	   (1 << 7)
#define SPI_RXNEIE	   (1 << 6)
#define SPI_ERRIE	   (1 << 5)
#define SPI_FRF		   (1 << 4)
#define SPI_NSSP	   (1 << 3)
#define SPI_SSOE	   (1 << 2)
#define SPI_TXDMAEN	   (1 << 1)
#define SPI_RXDMAEN	   (1 << 0)

//defining SR register
#define SPI_FTLVL(x)	(x << 11)
#define SPI_FRLVL(x)	(x << 9)
#define SPI_FRE		(1 << 8)
#define SPI_OVR		(1 << 6)
#define SPI_MODF	(1 << 5)
#define SPI_CRCERR	(1 << 4)
#define TXE		(1 << 1)
#define RXNE		(1 << 0)

//pointers to register
int* spi_cr1    = (int*) SPI_CR1;
int* spi_cr2    = (int*) SPI_CR2;
int* spi_sr     = (int*) SPI_SR ;
int* spi_dr1    = (int*) SPI_DR1 ;
int* spi_dr2    = (int*) SPI_DR2 ;
int* spi_dr3    = (int*) SPI_DR3 ;
int* spi_dr4    = (int*) SPI_DR4 ;
int* spi_dr5    = (int*) SPI_DR5 ;
int* spi_crcpr  = (int*) SPI_CRCPR;
int* spi_rxcrcr = (int*) SPI_RXCRCR;
int* spi_txcrcr = (int*) SPI_TXCRCR; 

struct shakti_spi {
	void __iomem      *regs;        /* virt. address of control registers */
	struct clk	  *clk;		/* bus clock */
	int 		  buffer_size;	/* buffer size in words */
	u32               cs_inactive;  /* level of the CS pins when inactive */
};

static void shakti_spi_write(struct shakti_spi *spi, int offset, u32 value)
{
	iowrite32(value, spi->regs + offset);
}

static u32 shakti_spi_read(struct shakti_spi *spi, int offset)
{
	return ioread32(spi->regs + offset);
}

static void shakti_spi_init(struct shakti_spi *spi)
{
	shakti_spi_write(spi, SPI_CR1 , (SPI_BR(7)|SPI_CPHA|SPI_CPOL));	/*setting up Baud Rate and sampling edge */
}

static int shakti_spi_prep_transfer(struct shakti_spi *spi, struct spi_device *device, struct spi_transfer *t)
{
	u32 cr;
	int mode;

	/* Modify the SPI protocol mode */
	cr = shakti_spi_read(spi, SPI_LSBFIRST); /* related to frame format - SPI_LSBFIRST*/

	/* LSB first? */
	cr &= ~SPI_LSBFIRST; 	//Use SPI_LSBFIRST
	if (device->mode & SPI_LSB_FIRST)
		cr |= SPI_LSBFIRST; //Use SPI_LSBFIRST
	
//check for return value and fn using it
}

//Need to update this file --- UPDATED but pending with some registers
static void shakti_spi_tx(struct shakti_spi *spi, const u8* tx_ptr)
{
	BUG_ON((shakti_spi_read(spi, SPI_SR) & SPI_FTLVL) != 0); // check FIFO full and its flag SPI_FTLVL
	shakti_spi_write(spi, SPI_SR, *tx_ptr & XSPI_DATA_MASK); //check with data mask
}

//Need to update the file --- UPDATED but pending with some registers
static void shakti_spi_rx(struct shakti_spi *spi, u8* rx_ptr)
{
        u32 data = shakti_spi_read(spi, SPI_SR);
        BUG_ON((data & SPI_FRLVL) != 0); // check FIFO empty and its flag SPI_FRLVL
        *rx_ptr = data & XSPI_DATA_MASK; //check with data mask	
}

//Need to update the file --- UPDATED --- Interrupt based check ///check
static void shakti_spi_wait(struct shakti_spi *spi, int bit, int poll)
{
	if (poll) {
		u32 cr;
		do cr = shakti_spi_read(spi, XSPI_IPR_OFFSET);		//TODO. understand
		while (!(cr & bit));
	} else {
		reinit_completion(&spi->done);
		shakti_spi_write(spi, XSPI_IER_OFFSET, bit);
		wait_for_completion(&spi->done);
	}
}

//Need to update the file --- UPDATED but pending with some registers
static void shakti_spi_execute(struct shakti_spi *spi, struct spi_transfer *t, int poll)
{
	int remaining_words = t->len;
	const u8* tx_ptr = t->tx_buf;
	u8* rx_ptr = t->rx_buf;

	while (remaining_words) {
		int n_words, tx_words, rx_words;
		n_words = min(remaining_words, spi->buffer_size);

		/* Enqueue n_words for transmission */
		for (tx_words = 0; tx_words < n_words; ++tx_words)
			shakti_spi_tx(spi, tx_ptr++);

		if (rx_ptr) {
			/* Wait for transmission + reception to complete */
			shakti_spi_write(spi, XSPI_RXWMR_OFFSET, n_words-1);	//TODO. understand regrads to interrupt
			shakti_spi_wait(spi, XSPI_RXWM_INTR, poll);		//TODO. understand

			/* Read out all the data from the RX FIFO */
			for (rx_words = 0; rx_words < n_words; ++rx_words)
				shakti_spi_rx(spi, rx_ptr++);
		} else {
			/* Wait for transmission to complete */
			shakti_spi_wait(spi, XSPI_TXWM_INTR, poll);
		}

		remaining_words -= n_words;
	}
}

static int shakti_spi_transfer_one(struct spi_master *master, struct spi_device *device, struct spi_transfer *t)
{
	struct shakti_spi *spi = spi_master_get_devdata(master);
	int poll;

	shakti_spi_prep_device(spi, device);
	poll = shakti_spi_prep_transfer(spi, device, t);
	shakti_spi_execute(spi, t, poll);

	return 0;
}

static int shakti_spi_probe(struct platform_device *pdev)
{
	struct shakti_spi *spi;
	struct resource *res;
	int ret, num_cs;
	u32 cs_bits, buffer_size, bits_per_word;
	struct spi_master *master;

	master = spi_alloc_master(&pdev->dev, sizeof(struct shakti_spi));
	if (!master) {
		dev_err(&pdev->dev, "out of memory\n");
		return -ENOMEM;
	}

	spi = spi_master_get_devdata(master);
	init_completion(&spi->done);
	platform_set_drvdata(pdev, master);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spi->regs)) {
		dev_err(&pdev->dev, "Unable to map IO resources\n");
		ret = PTR_ERR(spi->regs);
		goto put_master;
	}

	spi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(spi->clk)) {
		dev_err(&pdev->dev, "Unable to find bus clock\n");
		ret = PTR_ERR(spi->clk);
		goto put_master;
	}

	spi->irq = platform_get_irq(pdev, 0);
	if (spi->irq < 0) {
		dev_err(&pdev->dev, "Unable to find interrupt\n");
		ret = spi->irq;
		goto put_master;
	}

	/* Optional parameters */ ---- TODO Understand
	ret = of_property_read_u32(pdev->dev.of_node, "shakti,buffer-size", &buffer_size);
	if (ret < 0)
		spi->buffer_size = SHAKTI_SPI_DEFAULT_DEPTH;	//TODO. getting buffer depth of Shakti
	else
		spi->buffer_size = buffer_size;

	ret = of_property_read_u32(pdev->dev.of_node, "shakti,bits-per-word", &bits_per_word);
	if (ret < 0)
		bits_per_word = SHAKTI_SPI_DEFAULT_BITS;

	/* Spin up the bus clock before hitting registers */
	ret = clk_prepare_enable(spi->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable bus clock\n");
		goto put_master;
	}

	/* probe the number of CS lines */
	spi->cs_inactive = shakti_spi_read(spi, XSPI_CSDR_OFFSET);	//TODO. understand
	shakti_spi_write(spi, XSPI_CSDR_OFFSET, 0xffffffffU);		//TODO. understand
	cs_bits = shakti_spi_read(spi, XSPI_CSDR_OFFSET);		//TODO. understand
	shakti_spi_write(spi, XSPI_CSDR_OFFSET, spi->cs_inactive);	//TODO. understand
	if (!cs_bits) {
		dev_err(&pdev->dev, "Could not auto probe CS lines\n");
		ret = -EINVAL;
		goto put_master;
	}

	num_cs = ilog2(cs_bits) + 1;
	if (num_cs > SIFIVE_SPI_MAX_CS) {
		dev_err(&pdev->dev, "Invalid number of spi slaves\n");
		ret = -EINVAL;
		goto put_master;
	}

	/* Define our master - chenge wrt SHAKTI */
	master->bus_num = pdev->id;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST | SPI_CS_HIGH |
	                    SPI_TX_DUAL | SPI_TX_QUAD | SPI_RX_DUAL | SPI_RX_QUAD;
	master->flags = SPI_CONTROLLER_MUST_TX | SPI_MASTER_GPIO_SS;
	master->dev.of_node = pdev->dev.of_node;
	master->bits_per_word_mask = SPI_BPW_MASK(bits_per_word);
	master->num_chipselect = num_cs;
	master->transfer_one = shakti_spi_transfer_one;
	master->set_cs = shakti_spi_set_cs;

	/* Configure the SPI master hardware */
	shakti_spi_init(spi);

	dev_info(&pdev->dev, "mapped; irq=%d, cs=%d\n",
		spi->irq, master->num_chipselect);

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret < 0) {
		dev_err(&pdev->dev, "spi_register_master failed\n");
		goto put_master;
	}

	return 0;

put_master:
	spi_master_put(master);

	return ret;
}

static int shakti_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct shakti_spi *spi = spi_master_get_devdata(master);

	/* Disable all the interrupts just in case */
	shakti_spi_write(spi, XSPI_IER_OFFSET, 0);	//Not needed
	spi_master_put(master);

	return 0;
}

static const struct of_device_id shakti_spi_of_match[] = {
	{ .compatible = "shakti,spi0", },
	{}
};
MODULE_DEVICE_TABLE(of, shakti_spi_of_match);

static struct platform_driver shakti_spi_driver = {
	.probe = shakti_spi_probe,
	.remove = shakti_spi_remove,
	.driver = {
		.name = SHAKTI_SPI_NAME,
		.of_match_table = shakti_spi_of_match,
	},
};
module_platform_driver(shakti_spi_driver);

MODULE_AUTHOR("SandLogic Technologies Pvt Ltd");
MODULE_DESCRIPTION("ShaktiVajra SPI Linux driver");
MODULE_LICENSE("GPL");
