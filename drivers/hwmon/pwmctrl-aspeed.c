/*
 *
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/kernel.h>
#include <linux/mutex.h>

/* PWM register */
#define	PWM_GER_CTRL		0x00
#define PWM_DUTY_CTRL0		0x08
#define PWM_DUTY_CTRL1		0x0C
#define PWM_TACH_RESULT		0x2C
#define PWM_GER_EXT_CTRL	0x40
#define PWM_DUTY_CTRL2		0x48

/* PWM General control bit */
#define PWM_ENABLE_BIT	0x00000100  /* bit 8 */
#define PWM_ENABLE_ALL	0x00000F00

#define PWM_RESULT_VAL	0x000FFFFF
#define PWMTACH_CLOCK_RATE	24000000

/* PWM duty control register */
#define PWM_DUTY_ID_PWM			0x0000FFFF /* bits[15:0] */
#define PWM_DUTY_ID_SHIFT		16
#define PWM_DUTY_RISING			0
#define PWM_DUTY_FALLING		8


struct aspeed_pwm {
	int npwm;
	int ntach;
	struct device *dev;
	void __iomem *base;
	struct attribute **attrs;
	struct attribute_group group;
	const struct attribute_group *groups[2];
};

static int pwmtach_enable_pwm(struct aspeed_pwm *pwms, int index)
{
	u32 reg;
	
	//If pwm port number > 4,write to Gernerl externtion reg.
	if(index >=4) {
		reg = readl(pwms->base + PWM_GER_EXT_CTRL);
		reg |= (PWM_ENABLE_BIT << (index % 4));
		writel(reg, pwms->base + PWM_GER_EXT_CTRL);
	}
	else {
		reg = readl(pwms->base + PWM_GER_CTRL);
		reg |= (PWM_ENABLE_BIT<< index);
		writel(reg, pwms->base + PWM_GER_CTRL);
	}

	return 0;

}

static int pwmtach_disable_pwm(struct aspeed_pwm *pwms, int index)
{
	u32 reg;
	
	//If pwm port number > 4,write to Gernerl externtion reg.
	if(index >=4) {
		reg = readl(pwms->base + PWM_GER_EXT_CTRL);
		reg &= ~(PWM_ENABLE_BIT << (index % 4));
		writel(reg, pwms->base + PWM_GER_EXT_CTRL);
	}
	else {
		reg = readl(pwms->base + PWM_GER_CTRL);
		reg &= ~(PWM_ENABLE_BIT << index);
		writel(reg, pwms->base + PWM_GER_CTRL);
	}

	return 0;

}

static ssize_t pwmtach_set_pwm(struct device *dev, struct device_attribute *dev_attr, const char *buf, size_t count)
{
	struct aspeed_pwm *pwms= dev_get_drvdata(dev);
	struct sensor_device_attribute *snr_attr = to_sensor_dev_attr(dev_attr);

	u32 reg;
	u32 reg_base;
	u32 rising;
	u32 falling;
	int duty_val;

	if(kstrtoint(buf, 10, &duty_val))
		return -EINVAL;
	
	//Disable PWM when request duty value is 0
	if(duty_val == 0) {
		pwmtach_disable_pwm(pwms, snr_attr->index);
	}
	
	//Enable PWM port before write dutycycle value
	pwmtach_enable_pwm(pwms, snr_attr->index);	
	
	if(snr_attr->index >=4)
		reg_base = PWM_DUTY_CTRL2 + ((snr_attr->index / 6) * 4);
	else
		reg_base = PWM_DUTY_CTRL0 + ((snr_attr->index / 2) * 4);

	reg = readl(pwms->base + reg_base);

	rising =0;
	falling = (duty_val *255)/255;
	
	if (snr_attr->index & 0x01) { /* odd */
		rising <<= (PWM_DUTY_RISING + PWM_DUTY_ID_SHIFT);
		falling <<= (PWM_DUTY_FALLING + PWM_DUTY_ID_SHIFT);
		reg &= ~(PWM_DUTY_ID_PWM << PWM_DUTY_ID_SHIFT);
	} else { /* even */
		rising <<= PWM_DUTY_RISING;
		falling <<= PWM_DUTY_FALLING;
		reg &= ~PWM_DUTY_ID_PWM;
	}

	reg |= falling | rising;
	writel(reg, pwms->base + reg_base);
	
	return count;
}

static ssize_t pwmtach_pwm_reading(struct device *dev, struct device_attribute *dev_attr, char *buf)
{
	struct aspeed_pwm *pwms= dev_get_drvdata(dev);
	struct sensor_device_attribute *snr_attr = to_sensor_dev_attr(dev_attr);

	u32 reg;
	u32 reg_base;
	u32 rising;
	u32 falling;
	unsigned int dutycycle;
	int modulus = 0;	

	if(snr_attr->index >=4)
		reg_base = PWM_DUTY_CTRL2 + ((snr_attr->index / 6) * 4);
	else
		reg_base = PWM_DUTY_CTRL0 + ((snr_attr->index / 2) * 4);

	reg = readl(pwms->base + reg_base);
	
	if (snr_attr->index & 0x01) { /* odd */
		rising = (reg & 0x00ff0000) >> (PWM_DUTY_RISING + PWM_DUTY_ID_SHIFT);
		falling = (reg & 0xff000000)  >> (PWM_DUTY_FALLING + PWM_DUTY_ID_SHIFT);
	} else { /* even */
		rising = (reg & 0x000000ff)  >> PWM_DUTY_RISING;
		falling = (reg & 0x0000ff00)  >> PWM_DUTY_FALLING;
	}
	
	rising = rising*100 / 255;
	modulus = falling*100 % 255;
	falling = falling*100 / 255;
	if(falling == 0) {
		/* 256 is the pwm max */
		if(modulus != 0)
			dutycycle = 1;
		else
			dutycycle = 100;
	} 
	else 
	{	/* Round value up */
		if(modulus >= 5)
			falling++;
		
		dutycycle = falling - rising;
	}

	return sprintf(buf, "%d\n", dutycycle);
}

static ssize_t pwmtach_tach_reading(struct device *dev, struct device_attribute *dev_attr, char *buf)
{
	unsigned int ret_val;
	u32 reg;
	struct aspeed_pwm *pwms= dev_get_drvdata(dev);
	
	//Get all tach reading in register
	reg = readl(pwms->base + PWM_TACH_RESULT) & PWM_RESULT_VAL;
	if ((reg == 0x000FFFFE) || (reg == 0x000FFFFF) || (reg == 0))
		ret_val= 0;
	else	
		ret_val =  (PWMTACH_CLOCK_RATE * 60) / (2 * reg * 4);
	
	return sprintf(buf,"%d\n", ret_val);
}

static int aspeed_pwm_probe(struct platform_device *pdev)
{

	struct aspeed_pwm *pwms;
	struct sensor_device_attribute *attr;
	struct resource *res;
	int i,total;
	int pwm_port=1,tach_in=1;

	pwms = devm_kzalloc(&pdev->dev, sizeof(*pwms), GFP_KERNEL);
	if (!pwms)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pwms->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pwms->base)) 
		return PTR_ERR(pwms->base);

	pwms->dev = &pdev->dev;
	pwms->npwm = 8;
	pwms->ntach = pwms->npwm *2;

	//mutex_init(&pwms->lock);

	total = pwms->npwm + pwms->ntach;

	pwms->attrs = devm_kzalloc(&pdev->dev,
				 sizeof(*pwms->attrs) * (total+1),
				 GFP_KERNEL);

	if (pwms->attrs == NULL) {
		dev_err(&pdev->dev, "Failed to create attributes\n");		
		return -ENOMEM;
	}	

	for (i = 0; i < total; i++) {
		attr = devm_kzalloc(&pdev->dev, sizeof(*attr), GFP_KERNEL);
		if (attr == NULL) 
			return -ENOMEM;	
	
		sysfs_attr_init(&attr->dev_attr.attr);
		//Create PWM sensor attribute
		if(i<pwms->npwm) {
			attr->dev_attr.attr.name = kasprintf(GFP_KERNEL,
							  "pwm%d",
							  pwm_port++);

			attr->dev_attr.show = pwmtach_pwm_reading;
			attr->dev_attr.store = pwmtach_set_pwm;
			attr->dev_attr.attr.mode = S_IWUSR | S_IRUGO;
		}
		//Crete tach input sensor attribute
		else {  
			attr->dev_attr.attr.name = kasprintf(GFP_KERNEL,
							  "tach%d",
							  tach_in++);

			attr->dev_attr.show = pwmtach_tach_reading;
			attr->dev_attr.attr.mode = S_IRUGO;
		}
		attr->index = i;
		pwms->attrs[i] = &attr->dev_attr.attr;
	}

	pwms->group.attrs = pwms->attrs;
	pwms->groups[0] = &pwms->group;

	pwms->dev = devm_hwmon_device_register_with_groups(&pdev->dev, "pwmtach",
						       pwms, pwms->groups);

	if (IS_ERR(pwms->dev)) {
		dev_err(&pdev->dev, "Failed to register hwmon device\n");
		return PTR_ERR(pwms->dev);
	}

	platform_set_drvdata(pdev, pwms);

	dev_info(pwms->dev, "Aspeed PWM Tach driver\n");	


	return 0;

}

static int aspeed_pwm_remove(struct platform_device *pdev)
{
	struct aspeed_pwm *pwms = platform_get_drvdata(pdev);

	hwmon_device_unregister(pwms->dev);

	return 0;
}

static const struct of_device_id aspeed_pwm_of_match[] = {
	{ .compatible = "aspeed,aspeed-pwm", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, aspeed_pwm_of_match);

static struct platform_driver aspeed_pwm_driver = {
	.remove = aspeed_pwm_remove,	
	.driver = {
		.name = "aspeed-pwm",
		.of_match_table = aspeed_pwm_of_match,
	},
};


module_platform_driver_probe(aspeed_pwm_driver, aspeed_pwm_probe);

MODULE_DESCRIPTION("Aspeed PWM Tach driver");
MODULE_LICENSE("GPL v2");

