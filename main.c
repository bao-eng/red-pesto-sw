#include "pico/stdlib.h"

#define EASYSCALE_PIN 1
#define TRESET 2500
#define TSTART 50
#define TH_LB_US 50
#define TL_LB_US 150
#define TEOS 50
#define TVAL_ACKN 10
#define TACKN 512

void easyscale_reset() {
  gpio_put(EASYSCALE_PIN, 0);
  busy_wait_us(TRESET);
  gpio_put(EASYSCALE_PIN, 1);
}

void easyscale_start() {
  gpio_put(EASYSCALE_PIN, 1);
  busy_wait_us(TSTART);
}

void easyscale_eos() {
  gpio_put(EASYSCALE_PIN, 0);
  busy_wait_us(TEOS);
  gpio_put(EASYSCALE_PIN, 1);
}

void easyscale_write_bit(bool value) {
  if (value) {
    gpio_put(EASYSCALE_PIN, 0);
    busy_wait_us(TH_LB_US);
    gpio_put(EASYSCALE_PIN, 1);
    busy_wait_us(TL_LB_US);
  } else {
    gpio_put(EASYSCALE_PIN, 0);
    busy_wait_us(TL_LB_US);
    gpio_put(EASYSCALE_PIN, 1);
    busy_wait_us(TH_LB_US);
  }
}

int easyscale_set_vfb(uint8_t value, bool ack) {
  easyscale_start();
  bool buf[] = {0, 1, 1, 1, 0, 0, 1, 0};
  for (size_t i = 0; i < 8; i++) {
    easyscale_write_bit(buf[i]);
  }
  easyscale_eos();
  easyscale_start();
  bool buf1[] = {0, 0, 0};
  buf1[0] = ack;
  for (size_t i = 0; i < 3; i++) {
    easyscale_write_bit(buf1[i]);
  }
  for (size_t i = 4; i <= 0; i--)
  {
    easyscale_write_bit(value & (1<<i));
  }
  if(ack){
    gpio_put(EASYSCALE_PIN, 0);
    busy_wait_us(TVAL_ACKN);
    gpio_set_dir(EASYSCALE_PIN, GPIO_IN);
    bool tmp = gpio_get(EASYSCALE_PIN);
    busy_wait_us(TACKN);
    gpio_set_dir(EASYSCALE_PIN, GPIO_OUT);
    gpio_put(EASYSCALE_PIN, 1);
    return (int)tmp;
  }else{
    easyscale_eos();
    return 1;
  }
}

int main() {
  gpio_init(EASYSCALE_PIN);
  gpio_pull_up(EASYSCALE_PIN);
  gpio_set_dir(EASYSCALE_PIN, GPIO_OUT);
  gpio_put(EASYSCALE_PIN, 1);

  easyscale_reset();
  while (true) {
    for (size_t i = 0; i < 32; i++)
    {
        easyscale_set_vfb(i,true);
        sleep_ms(1000);
    }
  }
}
