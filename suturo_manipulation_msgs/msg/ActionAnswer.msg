# Undefined
uint8 UNDEFINED=0
# The robot has moved
uint8 SUCCESS=1
# There is no plan found
uint8 NOPLAN=2
# The movement failed
uint8 FAIL=3

# header for time/frame information
Header header

# the type (SUCCESS, NOPLAN, etc)
int32 type
