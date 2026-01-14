
uint8_t trama_GPS[1000];

uint8_t _getch1()
{
	while((UART1STA&1)==0);
	return UART1DAT;
}

void lectura_GPS(){	
	char cadenaUart[80];
	char buffer;
	char idAux[7];
	int i=0;
	int j, a; 
	char idSearch[] = {'$','G','N','G','G','A'}; //Entrada buscada
		 
	while(1){
		buffer = _getch1(); //Recojo char de la UART1	
		if(buffer == '$'){ //Indica inicio trama
		
			cadenaUart[0] ='$';
			idAux[0] = '$';
			for (i = 1; i < 80; i++){
				cadenaUart[i] = _getch1();
				if(cadenaUart[i] == '\n'){
					cadenaUart[i] = '\0';
					break; //Llegamos al final de una trama

				}
				
			}
			idAux[1] = cadenaUart[1];
			idAux[2] = cadenaUart[2];
			idAux[3] = cadenaUart[3];
			idAux[4] = cadenaUart[4];
			idAux[5] = cadenaUart[5];
			idAux[6] = '\0';
			
			if(mi_strcmp(idAux, idSearch) == 0){
				break;				
			}
			_printf("\nNo se ha encontrado la trama buscada");
			
		}	
	} 
	_printf("\nLa trama total recibida es: %s", cadenaUart);
	procesaCadena(cadenaUart, ',' ); 
		
}

int mi_strcmp(char *s1, char *s2) {

    while (*s1 != '\0' && *s2 != '\0' && *s1 == *s2) {
        s1++;
        s2++;
    }
    if (*s1 == *s2) {
        return 0;  /* Son cadenas iguales*/
    }
     else {
        return 1;  /*Son cadenas diferentes*/
    }
}

void procesaCadena(const char *cadena, const char delimitador ) {
    char token[50];  /*Tamaño máximo de un token, ajusta según tus necesidades*/
    int a = 0;
    while (*cadena) {
        int i = 0;
        while (*cadena != delimitador && *cadena != '\0') {
            token[i++] = *cadena++;
        }
        token[i] = '\0';
        switch(a) { /*Identificamos cada componente de la trama y enviamos*/
            case 0:
                _printf("Identificador: %s\n", token);
                break;
			case 1:
                _printf("UTC: %c%c:%c%c:%c%c\n", token[0], token[1], token[2], token[3], token[4], token[5]);
                break;
            case 2:
                 _printf("Latitud: %c%c.%c%c%c%c%c%c%c\n", token[0], token[1], token[2], token[3], token[5], token[6], token[7], token[8], token[9]);
                break;
            case 3:
                _printf("Dir. Latitud: %s\n", token);
                break;
            case 4:
				//_printf("Entero. Longitud: %s\n", token);
                _printf("Longitud: %c%c.%c%c%c%c%c%c%c \n", token[1], token[2], token[3], token[4], token[6], token[7], token[8], token[9], token[10]);
                break;
            case 5: 
                _printf("Dir. Longitud: %s\n", token);
                break;
            case 6:
                _printf("Status: %s\n", token);
                break;
            case 7:
                _printf("Satellites Used: %s\n", token);
                break;
			/* case 8:
                _printf("HDOP : %s\n", token);
                break; */
			case 9:
                _printf("Altitud : %s\n", token);
                break;
			/* case 10:
                _printf("Altura Elipsoide : %s\n", token);
                break;
			case 11:
                _printf("Tiempo Diferencial : %s\n", token);
                break;	
            default:
                _printf("Opción no válida.\n"); */
			default:
				break;
        }
        a++;
        if (*cadena != '\0') {
            cadena++;  /* Saltar el delimitador*/
        }
    }
}


static void gps_copy_str(char *dst, int dst_len, const char *src)
{
	int i = 0;

	if (dst_len <= 0) return;
	while (src[i] != '\0' && i < (dst_len - 1)) {
		dst[i] = src[i];
		i++;
	}
	dst[i] = '\0';
}

static int gps_is_gngga(const char *line)
{
	return (line[0] == '$' && line[1] == 'G' && line[2] == 'N' &&
	        line[3] == 'G' && line[4] == 'G' && line[5] == 'A');
}

int GPS_ReadLatLon(char *lat, int lat_len, char *lon, int lon_len)
{
	static char last_lat[16] = "0";
	static char last_lon[16] = "0";
	char line[80];
	char token[20];
	char lat_token[20] = "";
	char lon_token[20] = "";
	char fix = '0';
	char c;
	int i = 0;
	int field = 0;

	if ((UART1STA & 1) == 0) {
		gps_copy_str(lat, lat_len, last_lat);
		gps_copy_str(lon, lon_len, last_lon);
		return 0;
	}

	// Buscar inicio de trama.
	do {
		c = _getch1();
	} while (c != '$');

	line[0] = '$';
	for (i = 1; i < (int)(sizeof(line) - 1); i++) {
		c = _getch1();
		if (c == '\n') {
			break;
		}
		line[i] = c;
	}
	line[i] = '\0';

	if (!gps_is_gngga(line)) {
		gps_copy_str(lat, lat_len, last_lat);
		gps_copy_str(lon, lon_len, last_lon);
		return 0;
	}

	// Parsear campos separados por coma.
	i = 0;
	while (line[i] != '\0' && field <= 6) {
		int t = 0;
		while (line[i] != ',' && line[i] != '\0') {
			if (t < (int)(sizeof(token) - 1)) {
				token[t++] = line[i];
			}
			i++;
		}
		token[t] = '\0';

		if (field == 2) {
			gps_copy_str(lat_token, (int)sizeof(lat_token), token);
		} else if (field == 4) {
			gps_copy_str(lon_token, (int)sizeof(lon_token), token);
		} else if (field == 6) {
			fix = token[0];
		}

		if (line[i] == ',') i++;
		field++;
	}

	if (fix != '0' && lat_token[0] != '\0' && lon_token[0] != '\0') {
		gps_copy_str(last_lat, (int)sizeof(last_lat), lat_token);
		gps_copy_str(last_lon, (int)sizeof(last_lon), lon_token);
	}

	gps_copy_str(lat, lat_len, last_lat);
	gps_copy_str(lon, lon_len, last_lon);
	return (fix != '0');
}
