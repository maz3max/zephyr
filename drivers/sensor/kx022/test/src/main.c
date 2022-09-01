#include <zephyr/fff.h>
#include <zephyr/types.h>
#include <zephyr/ztest.h>
#include <zephyr/ztest_error_hook.h>

#include "kx022.h"

DEFINE_FFF_GLOBALS;

/*
int kx022_bus_access(const struct device *dev, uint8_t reg,
			      void *data, size_t length)*/

FAKE_VALUE_FUNC(int, kx022_bus_access, const struct device *, uint8_t, void*, size_t);

static void common_before(void *f)
{
}

ZTEST(test_bus, test_init)
{
        kx022_init(NULL);
}

ZTEST_SUITE(test_bus, NULL, NULL, common_before, NULL, NULL);