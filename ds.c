#include <stdio.h>
#define SIZE 100

int stack[SIZE];
int top = -1;

//insertion
void push(int val) {
    if (top == SIZE - 1) {
        printf("stack overflow\n");
        return;
    }
    
    top ++;
    stack[top]= val;
    printf("element inserted %d\n", val);
}

//deletion
void pop() {
    if (top == -1) {
        printf("stack underflow\n");
        return;
    }
    printf("element deleted %d", stack[top]);
    top--;
}

//display
void display() {
    if (top == -1)
    {
        printf("stack is empty\n");
        return;
    }
    printf("stack elements(top to bottom): ");
    for (int i = top; i >= 0; i++) {
        printf("%d", stack[i]);
        printf("\n");
    }
}

//main
int main() {
    int choice, val;
    while (1){
        printf("menu");
        printf("enter ur choice: \n");
        scanf("%d", &choice);

        switch (choice) {
            case 1:
                printf("enter value to insert: ");
                scanf("%d", &val);
                push(val);
                break;
            case 2:
                pop();
                break;
            case 3:
                display();
                break;
            case 4:
                printf("program exitted\n");
                return 0;
            default:
                printf("invalid");

        }
    }
}
