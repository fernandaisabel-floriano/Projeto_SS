##Finite Impulse Response pra um filtro passa banda que vai de 1400 a 3200 Hz
pkg load signal

Fs = 20000
fz_menor1 = 1280;
fz_maior1 = 1720;

fz_menor2 = 2100;
fz_maior2 = 2540;

fz_menor3 = 2920;
fz_maior3 = 3360;
ordem = 150;
Nyquist = Fs / 2;

fmenor_norm1 = fz_menor1/Nyquist;
fMaior_norm1 = fz_maior1/Nyquist;

fmenor_norm2 = fz_menor2/Nyquist;
fMaior_norm2 = fz_maior2/Nyquist;

fmenor_norm3 = fz_menor3/Nyquist;
fMaior_norm3 = fz_maior3/Nyquist;

gama1 = [fmenor_norm1,fMaior_norm1];
gama2 = [fmenor_norm2,fMaior_norm2];
gama3 = [fmenor_norm3,fMaior_norm3];

h1 = fir1(ordem,gama1,'bandpass');
h2 = fir1(ordem,gama2,'bandpass');
h3 = fir1(ordem,gama3,'bandpass');


printf("h1\n");

for i = 1:length(h1)
    printf("%.9f", h1(i));
    if (i < length(h1))
        printf(", ");
    endif
    # Quebra de linha a cada 5 números para o código não ficar infinito para o lado
    if (mod(i, 5) == 0 && i < length(h1))
        printf("\n    ");
    endif
endfor
printf("\n};\n");

printf("h2\n");
for i = 1:length(h2)
    printf("%.9f", h2(i));
    if (i < length(h2))
        printf(", ");
    endif
    # Quebra de linha a cada 5 números para o código não ficar infinito para o lado
    if (mod(i, 5) == 0 && i < length(h1))
        printf("\n    ");
    endif
endfor
printf("\n};\n");

printf("h3\n");
for i = 1:length(h3)
    printf("%.9f", h3(i));
    if (i < length(h3))
        printf(", ");
    endif
    # Quebra de linha a cada 5 números para o código não ficar infinito para o lado
    if (mod(i, 5) == 0 && i < length(h1))
        printf("\n    ");
    endif
endfor
printf("\n};\n");


