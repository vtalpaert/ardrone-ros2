#include <stdlib.h>
#include <libARCommands/ARCOMMANDS_Generator.h>

int main (int argc, char *argv[])
{
    uint8_t* buffer = NULL;
    int32_t buffLen = 1;
    int32_t* cmdLen = NULL;
    eARCOMMANDS_GENERATOR_ERROR err = ARCOMMANDS_Generator_GenerateGenericDefault(buffer, buffLen, cmdLen);
    return 0;
}
