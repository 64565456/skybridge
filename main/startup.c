#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "esp_ota_ops.h"

void startup_print(void)
{
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    if (running_partition)
    {
        printf("Running Partition:\n");
        printf("  type: %d\n", running_partition->type);
        printf("  subtype: %d\n", running_partition->subtype);
        printf("  address: 0x%08lx\n", running_partition->address);
        printf("  size: %lu\n", running_partition->size);
        printf("  erase_size: %lu\n", running_partition->erase_size);
        printf("  label: %s\n", running_partition->label);
        printf("  encrypted: %d\n", running_partition->encrypted);
    }
    else
    {
        printf("Running Partition: NULL\n");
    }

    if (update_partition)
    {
        printf("Update Partition:\n");
        printf("  type: %d\n", update_partition->type);
        printf("  subtype: %d\n", update_partition->subtype);
        printf("  address: 0x%08lx\n", update_partition->address);
        printf("  size: %lu\n", update_partition->size);
        printf("  erase_size: %lu\n", update_partition->erase_size);
        printf("  label: %s\n", update_partition->label);
        printf("  encrypted: %d\n", update_partition->encrypted);
    }
    else
    {
        printf("Update Partition: NULL\n");
    }
}

