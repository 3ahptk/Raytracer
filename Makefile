.SUFFIXES: .c .o
CC = gcc
EXEC = ray
<<<<<<< HEAD
CCFLAGS = -Wall -Wextra
=======
CCFLAGS = -Wall -std=gnu99
>>>>>>> 8a5c03482b0829c425c5dcf66e60ec3dbed0ea27
SRC = raytracer.c
OBJS = raytracer.o
OUT = reference.png custom.png

${EXEC}: ${OBJS}
	${CC} ${CCFLAGS} -lm -o ${EXEC} ${OBJS}

${OBJS}:
	${CC} ${CCFLAGS} -c ${SRC}

run: ${EXEC}
	./${EXEC}

clean:
	rm -f ${EXEC} ${OBJS} ${OUT}

raytracer.o:raytracer.c
