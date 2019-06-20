#include <stdio.h>

int main(void){
	FILE *pfile = NULL;

	pfile = fopen("hw2.txt","r");
	if(pfile == NULL){		
		return 0;
	}

	int n,i;
    double x1, y1;

    fscanf(pfile,"%d",&n);
    printf("%d\n",n);	

for(i = 0; i < n ; i++){

    fscanf(pfile,"%lf%lf",&x1,&y1);
    printf("%lf %lf\n",x1,y1);
}

	fclose(pfile);
	return 0;
}
