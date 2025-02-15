//按第一行展开计算|A|
static float getA(float arcs[3][3], int n)
{
    if(n==1) {
        return arcs[0][0];
    }
    float ans = 0;
    float temp[3][3]={{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
    int i,j,k;
    for(i=0;i<n;i++)
    {
        for(j=0;j<n-1;j++)
        {
            for(k=0;k<n-1;k++)
            {
                temp[j][k] = arcs[j+1][(k>=i)?k+1:k];
                
            }
        }
        float t = getA(temp,n-1);
        if(i%2==0)
        {
            ans += arcs[0][i]*t;
        }
        else
        {
            ans -=  arcs[0][i]*t;
        }
    }
    return ans;
}

//计算每一行每一列的每个元素所对应的余子式，组成A*
static void getAStart(float arcs[3][3], int n, float ans[3][3])
{
    if(n==1)
    {
        ans[0][0] = 1;
        return;
    }
    int i,j,k,t;
    float temp[3][3];
    for(i=0;i<n;i++)
    {
        for(j=0;j<n;j++)
        {
            for(k=0;k<n-1;k++)
            {
                for(t=0;t<n-1;t++)
                {
                    temp[k][t] = arcs[k>=i?k+1:k][t>=j?t+1:t];
                }
            }
            
            
            ans[j][i]  =  getA(temp,n-1);  //此处顺便进行了转置
            if((i+j)%2 == 1)
            {
                ans[j][i] = - ans[j][i];
            }
        }
    }
}

//得到给定矩阵src的逆矩阵保存到des中。
int matrix_inverse(float src[3][3], int n, float des[3][3])
{
    float flag=getA(src,n);
    float t[3][3];
    if(0==flag)
    {
        // cout<< "原矩阵行列式为0，无法求逆。请重新运行" <<endl;
        return 0;//如果算出矩阵的行列式为0，则不往下进行
    }
    else
    {
        getAStart(src,n,t);
        for(int i=0;i<n;i++)
        {
            for(int j=0;j<n;j++)
            {
                des[i][j]=t[i][j]/flag;
            }
            
        }
    }
    return 1;
}

void matrix_mult(float matrix1[3][3], float matrix2[3][3], float ans[3][3])
{
	int i, j,k;
	for (i = 0; i < 3; i++)
	{
		for(j=0;j<3;j++){
            for(k=0;k<3;k++){
                ans[i][j]=ans[i][j]+matrix1[i][k]*matrix2[k][j]; 
            } 
        }
	}
}