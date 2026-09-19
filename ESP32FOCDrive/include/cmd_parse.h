#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

/**
 * One command line: verb, optional axis prefix, optional numeric value.
 * Header-only and Arduino-free so the Unity suite compiles without src/.
 */
struct ParsedCmd {
  char cmd[12];
  uint8_t axis_mask;
  bool has_value;
  float value;
};

inline const char *cmdSkipSpaces(const char *s) {
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  return s;
}

inline bool parseCmd(const char *line, ParsedCmd *out) {
  memset(out, 0, sizeof(*out));
  out->axis_mask = (uint8_t)FOC_AXIS_MASK;

  const char *s = cmdSkipSpaces(line);
  if (*s == '\0') {
    return false;
  }
  size_t n = 0;
  while (*s != '\0' && *s != ' ' && *s != '\t') {
    if (n + 1u < sizeof(out->cmd)) {
      out->cmd[n++] = *s;
    }
    ++s;
  }
  out->cmd[n] = '\0';

  s = cmdSkipSpaces(s);
  if (*s == 'L' || *s == 'l') {
    out->axis_mask = 0b01;
    ++s;
  } else if (*s == 'R' || *s == 'r') {
    out->axis_mask = 0b10;
    ++s;
  }

  s = cmdSkipSpaces(s);
  if (*s != '\0') {
    char *end = nullptr;
    const float v = strtof(s, &end);
    if (end != s) {
      out->has_value = true;
      out->value = v;
    }
  }
  return true;
}
