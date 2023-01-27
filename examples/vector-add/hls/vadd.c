void vadd(int a[100], int b[100], int c[100]) {
#pragma HLS INTERFACE m_axi port=a
#pragma HLS INTERFACE m_axi port=b
#pragma HLS INTERFACE m_axi port=c
#pragma HLS INTERFACE s_axilite port=return
    for(int i = 0; i < 100; i++) {
        c[i] = a[i] + b[i];
    }
}
