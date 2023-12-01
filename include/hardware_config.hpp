struct gpio_mux_led
{
    uint8_t gpio_1=0;
    uint8_t gpio_2=1;
    uint8_t gpio_3=2;
    uint8_t gpio_4=3;
    uint8_t gpio_5=4;
    uint8_t gpio_6=5;
    uint8_t gpio_7=6;
    uint8_t gpio_8=7;
    uint8_t EN1=0;
    uint8_t EN2=14;
};


struct gpio_mux_SW
{
    uint8_t s0=14;
    uint8_t s1=15;
    uint8_t s2=0;
    uint8_t s3=1;
    uint8_t en=6;
    uint8_t state=5;   
};

struct gpio_mux_ETH
{
    uint8_t gpio_1=2;
    uint8_t gpio_2=3;
    uint8_t gpio_3=4;
    uint8_t gpio_4=5; 
    uint8_t en=7;   
    uint8_t in_out=4;
    /*****************/
    uint8_t in_0=0;
    uint8_t in_1=1;
    uint8_t in_2=2;
    uint8_t in_3=3; 
    uint8_t in_4=4;
    uint8_t in_5=5;
    uint8_t in_6=6;
    uint8_t in_7=7;
};

struct gpio_lcd
{
    uint8_t RS=10;//
    uint8_t RW=11;//
    uint8_t EN=12;//
    uint8_t data_4=6;
    uint8_t data_5=7;
    uint8_t data_6=8;
    uint8_t data_7=9;
};

struct gpio_ina337
{
    uint8_t gpio_1=0;
    uint8_t gpio_2=1;
    uint8_t gpio_3=2;
    uint8_t gpio_4=3;
    uint8_t gpio_5=4;
    uint8_t gpio_6=5;
    uint8_t gpio_7=6;
    uint8_t gpio_8=7;
};

struct gpio_SPI
{
    uint8_t miso=11;
    uint8_t mosi=12;
    uint8_t cs=13;
    uint8_t sck=10;
    uint8_t ce=2;
};

struct gpio_usart
{
    uint8_t TX=9;
    uint8_t RX=10;   
};

struct gpio_i2c
{
    uint8_t SCL=8;
    uint8_t SDA=9;   
};

struct gpio_swithes
{
    uint8_t EN_1=5;
    uint8_t EN_2=6;
    uint8_t EN_3=7;
    uint8_t EN_4=8;
    uint8_t EN_5=9;
    uint8_t EN_6=10;
    uint8_t EN_7=11;
    uint8_t EN_8=12; 
};