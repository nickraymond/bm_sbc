/// @file test_transport_kind.c
/// @brief Unit tests for the transport-name parser.

#include "transport_kind.h"

#include <stdio.h>
#include <string.h>

static int g_pass = 0;
static int g_fail = 0;

#define ASSERT_EQ(a, b, msg)                                                   \
  do {                                                                         \
    if ((a) != (b)) {                                                          \
      printf("  FAIL: %s (got %d, expected %d)\n", msg, (int)(a), (int)(b));   \
      g_fail++;                                                                \
    } else {                                                                   \
      g_pass++;                                                                \
    }                                                                          \
  } while (0)

static void test_parse_valid(void) {
  TransportKind k = TransportAdin;
  ASSERT_EQ(transport_kind_parse("virtual", &k), true, "parse virtual ok");
  ASSERT_EQ(k, TransportVirtual, "virtual value");
  ASSERT_EQ(transport_kind_parse("udp", &k), true, "parse udp ok");
  ASSERT_EQ(k, TransportUdp, "udp value");
  ASSERT_EQ(transport_kind_parse("serial", &k), true, "parse serial ok");
  ASSERT_EQ(k, TransportSerial, "serial value");
  ASSERT_EQ(transport_kind_parse("adin", &k), true, "parse adin ok");
  ASSERT_EQ(k, TransportAdin, "adin value");
}

static void test_parse_invalid(void) {
  TransportKind k = TransportVirtual;
  ASSERT_EQ(transport_kind_parse("", &k), false, "empty rejected");
  ASSERT_EQ(transport_kind_parse("UDP", &k), false, "case-sensitive");
  ASSERT_EQ(transport_kind_parse("ethernet", &k), false, "unknown rejected");
  ASSERT_EQ(transport_kind_parse(NULL, &k), false, "NULL name rejected");
  ASSERT_EQ(transport_kind_parse("udp", NULL), false, "NULL out rejected");
  ASSERT_EQ(k, TransportVirtual, "out untouched on failure");
}

static void test_names(void) {
  ASSERT_EQ(strcmp(transport_kind_name(TransportVirtual), "virtual"), 0,
            "name virtual");
  ASSERT_EQ(strcmp(transport_kind_name(TransportUdp), "udp"), 0, "name udp");
  ASSERT_EQ(strcmp(transport_kind_name(TransportSerial), "serial"), 0,
            "name serial");
  ASSERT_EQ(strcmp(transport_kind_name(TransportAdin), "adin"), 0,
            "name adin");
}

int main(void) {
  printf("transport_kind tests:\n");
  test_parse_valid();
  test_parse_invalid();
  test_names();
  printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
