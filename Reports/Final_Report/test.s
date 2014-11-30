main:	# assembly code		# effect			
		lb $2, 80($0)		# $2 = 5			
		lb $7, 76($0)		# $7 = 3			
		lb $3, 81($7)		# $3 = 12			
		or $4, $7, $2		# $4 <=3 or 5 = 7	
		and $5, $3, $4		# $5 <=12 and 7 = 4	
		add $5, $5, $4		# $5 <= 4+7 = 11	
		beq $5, $7, end		# not taken			
		slt $6, $3, $4		# $6 <= 12 < 7 = 0	
		beq $6, $0, around	# not taken			
		lb $5, 0($0)		# not taken			
around:	slt $6, $7, $2		# $6 <= 3 < 5 = 1	
		add $7, $6, $5		# $7 <= 1 + 11 = 12	
		sub $7, $7, $2		# $7 <= 12 - 5 = 7	
		add $3, $7, $0		# $3 = 7			
loop:	beq $3, $0, end		# taken if $3 = 0	
		sub $3, $3, $6      # $3 = $3 - 1       
		j loop              # taken             
		j end               # not taken         
end:	sb $7, 71($2)       # write adr 76 <=7  
		.dw	3                                   
		.dw 5                                   
		.dw 12	                                

