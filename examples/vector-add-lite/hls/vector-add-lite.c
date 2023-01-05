void vector_add(int a[10], int b[10], int c[10]) {
#pragma HLS INTERFACE s_axilite port=a
#pragma HLS INTERFACE s_axilite port=b
#pragma HLS INTERFACE s_axilite port=c
#pragma HLS INTERFACE s_axilite port=return
    for (int i = 0; i < 10; i++) {
        c[i] = a[i] + b[i];
    }
}
