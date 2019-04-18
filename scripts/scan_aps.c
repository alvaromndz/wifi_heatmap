#include <stdio.h>
#include <stdlib.h>
#include <iwlib.h>
#include <assert.h>

int main(int argc, char *argv[]) {
  assert(argc == 2);
  wireless_scan_head head;
  wireless_scan *result;
  iwrange range;
  int sock;
  char buffer[128]; /* Temporary buffer */

  /* Open socket to kernel */
  sock = iw_sockets_open();

  /* Get some metadata to use for scanning */
  if (iw_get_range_info(sock, argv[1], &range) < 0) {
    printf("Error during iw_get_range_info. Aborting.\n");
    exit(2);
  }

  /* Perform the scan */
  if (iw_scan(sock, "wlp2s0", range.we_version_compiled, &head) < 0) {
    printf("Error during iw_scan. Aborting.\n");
    exit(2);
  }

  /* Traverse the results */
  result = head.result;
  while (NULL != result) {
    if (result->has_stats && result->b.has_essid && result->has_ap_addr){
	   printf("%s, %s, %u\n", result->b.essid, 
			   iw_sawap_ntop(&result->ap_addr, buffer),
			   result->stats.qual.level);
    } 
    result = result->next;
  }
  return EXIT_SUCCESS;
}
