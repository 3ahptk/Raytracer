.SUFFIXES: .c .o
CC = gcc
EXEC = ray
CCFLAGS = -Wall
SRC = raytracer.c
OBJS = raytracer.o

${EXEC}: ${OBJS}
	${CC} ${CCFLAGS} -lm -o ${EXEC} ${OBJS}

${OBJS}:
	${CC} ${CCFLAGS} -c ${SRC}

run: ${EXEC}
	./${EXEC}

clean:
	rm -f ${EXEC} ${OBJS}

raytracer.o:raytracer.c
