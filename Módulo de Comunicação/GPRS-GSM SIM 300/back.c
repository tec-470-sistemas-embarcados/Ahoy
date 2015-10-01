/*
		if(INTCONbits.TMR0IF){

			INTCONbits.TMR0IF = 0;		// Limpa o vetor de interrupção
			
			// Desligar motores
			if( index_ant >= 0 ){
	
				for( i = index_ant; i < index_atual; i++ ){
					
					switch( motor[i] ){
						case 0:
							MOTOR_1 = 0;
							break;
						case 1:
							MOTOR_2 = 0;
							break;
						case 2:
							MOTOR_3 = 0;
							break;
						case 3:
							MOTOR_4 = 0;
							break;
					}
				}
			}
			
			if( index_atual == 0 ){		//	Inicio do ciclo
				
				if(novo_dado){
					for( i = 0; i < 4; i++ ){
						T[i] = novo_T[i];
						motor[i] = novo_motor[i];
					}

					novo_dado = FALSE;
				}
				
				MOTOR_1 = 1;
				MOTOR_2 = 1;
				MOTOR_3 = 1;
				MOTOR_4 = 1;

				delta = T[0];

				index_ant = 0;
				index_atual = index_atual + 1;
			}
			else{	// indice maior do que zero.
				
				if( index_atual < 4 ){	// index entre 1 e 3

					for( i = index_atual; i < 4; i++ ){
						
						delta = T[i] - T[index_ant];
	
						if( delta > 0){
							index_ant = index_atual;	// passa ao inicio da cadeia
							index_atual = i + 1;		// passa para o próximo slot após o fim da cadeia

							break;
						}
					}

					if( delta == 0 ){	// Significa que o fim do vetor foi atingido com valores iguais
						index_ant = index_atual;
						index_atual = 4;	// fim do vetor

						for( i = index_ant; i < index_atual; i++ ){
							
							switch( motor[i] ){
								case 0:
									MOTOR_1 = 0;
									break;
								case 1:
									MOTOR_2 = 0;
									break;
								case 2:
									MOTOR_3 = 0;
									break;
								case 3:
									MOTOR_4 = 0;
									break;
							}
						}

						index_ant = -1;
						index_atual = 0;

						delta = period - T[4]; 
					}
				}
				else{	// Caso tenha chegado ao fim do vetor
					index_ant = -1;
					index_atual = 0;

					delta = period - T[4];		// Calculo do periodo em baixo
				}
			}

			WriteTimer0(65536 - delta);			// Escreve o Delta			
		}
		*/