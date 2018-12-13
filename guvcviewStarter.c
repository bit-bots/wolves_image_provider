#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>

void *startGUVC(void *path){
	char command[100] = "guvcview --profile=";
	strcat(command, (char*)path);
	system(command);
	return;
}

int main(int argc, char *argv[]){
	if(argc <2){
		printf("Usage: %s path_to_profile_file\n", argv[0]);
		return 1;
	}

	pthread_t start;
	int threaad = pthread_create(&start, NULL, &startGUVC, (void*)argv[1]);
	usleep(1000000);
	pthread_cancel(start);
	system("killall guvcview");

	return 0;
}
