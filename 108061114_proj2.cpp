#include <iostream>
#include <fstream>
#include <queue>
#include <stack>
#include <algorithm>
#include <ctime>
using namespace std;

#define UP 'a'
#define DOWN 'b'
#define LEFT 'c'
#define RIGHT 'd'

class cell;

class robot {
    public :
        int life;
        int rem_life;       // remaining battery life
};

class floor {
    public :
        int size_r, size_c;     // size of floor's rows and columns
        int num_need_clean;     // the number of the cells that need to be cleaned
        int R_i, R_j;
        int total_step;
        
        floor(void) : num_need_clean(0), total_step(0) {}
        void map_dis(cell **);
        void clean(robot *, cell **, int, int, queue<int> *, queue<int> *);
        char find_path(robot *, cell **, int, int);
        void BackHome(robot *, cell **, int *, int *, queue<int> *, queue<int> *);
};

class cell {
    public :
        char type;      // obstacle, floor, or recharging place
        bool whether_clean;
        int dis;

        cell(void) : whether_clean(0), dis(-1) {}
};

int main(int argc, char **argv)
{
    fstream fin, fout;
    floor Floor;
    robot Robot;
    char temp;
    cell **backgnd;
    int i = 0, j = 0;
    queue<int> row, col;

    fin.open(argv[1], ios::in);
    fin >> Floor.size_r >> Floor.size_c >> Robot.life;
    Robot.rem_life = Robot.life;
    backgnd = new cell * [Floor.size_r];
    for (int index = 0; index < Floor.size_r; index++)
        backgnd[index] = new cell [Floor.size_c];
    int N;
    for (N = 1, fin >> temp; N <= Floor.size_r * Floor.size_c; fin >> temp, N++) {
        backgnd[i][j].type = temp;
        if (temp != '1')
            Floor.num_need_clean++;
        if (temp == 'R') {
            Floor.R_i = i;
            Floor.R_j = j;
        }
        if (j == Floor.size_c - 1) {
            j = 0;
            i++;
        } else {
            j++;
        }
    }
    fin.close();

    Floor.map_dis(backgnd);
    fout.open("final.path", ios::out);
    for (int i = 0; i < Floor.size_r && Floor.num_need_clean > 0; i++) {
        for (int j = 0; j < Floor.size_c && Floor.num_need_clean > 0; j++) {
            if (backgnd[i][j].whether_clean == 0 && backgnd[i][j].type == '0') {
                Floor.clean(&Robot, backgnd, i, j, &row, &col);
            }
        }
    }

    Floor.total_step = row.size();
    fout << Floor.total_step << endl;
    fout << Floor.R_i << ' ' << Floor.R_j << endl;
    while(!row.empty()) {
        fout << row.front() << ' ' << col.front() << endl;
        row.pop();
        col.pop();
    }
    fout.close();

    for (int i = 0; i < Floor.size_r; i++) {
        delete [] backgnd[i];
    }
    delete [] backgnd;

    return 0;
}

void floor::map_dis(cell **backgnd) {
    queue<int> row, col;
    int i = R_i, j = R_j;

    backgnd[R_i][R_j].dis = 0;
    row.push(R_i);
    col.push(R_j);
    while (1) {
        row.pop();
        col.pop();
        if (i - 1 >= 0 && backgnd[i - 1][j].dis == -1 && backgnd[i - 1][j].type != '1') {
            row.push(i - 1);
            col.push(j);
            backgnd[i - 1][j].dis = backgnd[i][j].dis + 1;
        }
        if (i + 1 < size_r && backgnd[i + 1][j].dis == -1 && backgnd[i + 1][j].type != '1') {
            row.push(i + 1);
            col.push(j);
            backgnd[i + 1][j].dis = backgnd[i][j].dis + 1;
        }
        if (j - 1 >= 0 && backgnd[i][j - 1].dis == -1 && backgnd[i][j - 1].type != '1') {
            row.push(i);
            col.push(j - 1);
            backgnd[i][j - 1].dis = backgnd[i][j].dis + 1;
        }
        if (j + 1 < size_c && backgnd[i][j + 1].dis == -1 && backgnd[i][j + 1].type != '1') {
            row.push(i);
            col.push(j + 1);
            backgnd[i][j + 1].dis = backgnd[i][j].dis + 1;
        }

        if (row.empty())
            break;
        i = row.front();
        j = col.front();
    }
}

void floor::clean(robot *Robot, cell **backgnd, int target_i, int target_j, queue<int> *r, queue<int> *c) {
    int i, j;
    bool blind_alley;
    char mode;
    stack<int> row, col;

    row.push(target_i);
    col.push(target_j);
    while (row.top() != R_i || col.top() != R_j) {
        mode = find_path(Robot, backgnd, row.top(), col.top());
        if (mode == UP) {
            row.push(row.top() - 1);
            col.push(col.top());
        } else if (mode == LEFT) {
            row.push(row.top());
            col.push(col.top() - 1);
        } else if (mode == DOWN) {
            row.push(row.top() + 1);
            col.push(col.top());
        } else {
            row.push(row.top());
            col.push(col.top() + 1);
        }
        Robot->rem_life--;
    }
    while (!row.empty()) {
        i = row.top();
        j = col.top();
        if (i != R_i || j != R_j) {
            r->push(i);
            c->push(j);
        }
        if (backgnd[i][j].whether_clean == 0) {
            backgnd[i][j].whether_clean = 1;
            num_need_clean--;
        }
        row.pop();
        col.pop();
    }
    
    i = target_i;
    j = target_j;
    while(1) {
        blind_alley = true;
        if (j + 1 < size_c && backgnd[i][j + 1].type != '1' &&
            backgnd[i][j + 1].whether_clean == 0 && Robot->rem_life >= backgnd[i][j + 1].dis + 1) {
            Robot->rem_life--;
            j++;
            num_need_clean--;
            backgnd[i][j].whether_clean = 1;
            r->push(i);
            c->push(j);
            blind_alley = false;
        } else if (i + 1 < size_r && backgnd[i + 1][j].type != '1' &&
                   backgnd[i + 1][j].whether_clean == 0 && Robot->rem_life >= backgnd[i + 1][j].dis + 1) {
            Robot->rem_life--;
            i++;
            num_need_clean--;
            backgnd[i][j].whether_clean = 1;
            r->push(i);
            c->push(j);
            blind_alley = false;
        } else if (j - 1 >= 0 && backgnd[i][j - 1].type != '1' &&
                   backgnd[i][j - 1].whether_clean == 0 && Robot->rem_life >= backgnd[i][j - 1].dis + 1) {
            Robot->rem_life--;
            j--;
            num_need_clean--;
            backgnd[i][j].whether_clean = 1;
            r->push(i);
            c->push(j);
            blind_alley = false;
        } else if (i - 1 >= 0 && backgnd[i - 1][j].type != '1' &&
                   backgnd[i - 1][j].whether_clean == 0 && Robot->rem_life >= backgnd[i - 1][j].dis + 1) {
            Robot->rem_life--;
            i--;
            num_need_clean--;
            backgnd[i][j].whether_clean = 1;
            r->push(i);
            c->push(j);
            blind_alley = false;
        }

        if (blind_alley) {
            BackHome(Robot, backgnd, &i, &j, r, c);
        }

        if (i == R_i && j == R_j) {
            Robot->rem_life = Robot->life;
            break;
        }
    }
}

char floor::find_path(robot *Robot, cell **backgnd, int i, int j) {
    if (j + 1 < size_c && backgnd[i][j + 1].type != '1' && backgnd[i][j + 1].whether_clean == '0' && backgnd[i][j + 1].dis < backgnd[i][j].dis) {
        return RIGHT;
    }
    if (i + 1 < size_r && backgnd[i + 1][j].type != '1' && backgnd[i + 1][j].whether_clean == '0' && backgnd[i + 1][j].dis < backgnd[i][j].dis) {
        return DOWN;
    }
    if (j - 1 >= 0 && backgnd[i][j - 1].type != '1' && backgnd[i][j - 1].whether_clean == '0' && backgnd[i][j - 1].dis < backgnd[i][j].dis) {
        return LEFT;
    }
    if (i - 1 >= 0 && backgnd[i - 1][j].type != '1' && backgnd[i - 1][j].whether_clean == '0' && backgnd[i - 1][j].dis < backgnd[i][j].dis) {
        return UP;
    }
   
    if (j + 1 < size_c && backgnd[i][j + 1].type != '1' && backgnd[i][j + 1].whether_clean == '0' && backgnd[i][j + 1].dis <= Robot->rem_life - 1) {
        return RIGHT;
    }
    if (i + 1 < size_r && backgnd[i + 1][j].type != '1' && backgnd[i + 1][j].whether_clean == '0' && backgnd[i + 1][j].dis <= Robot->rem_life - 1) {
        return DOWN;
    }
    if (j - 1 >= 0 && backgnd[i][j - 1].type != '1' && backgnd[i][j - 1].whether_clean == '0' && backgnd[i][j - 1].dis <= Robot->rem_life - 1) {
        return LEFT;
    }
    if (i - 1 >= 0 && backgnd[i - 1][j].type != '1' && backgnd[i - 1][j].whether_clean == '0' && backgnd[i - 1][j].dis <= Robot->rem_life - 1) {
        return UP;
    }

    if (j + 1 < size_c && backgnd[i][j + 1].type != '1' && backgnd[i][j + 1].dis < backgnd[i][j].dis) {
        return RIGHT;
    }
    if (i + 1 < size_r && backgnd[i + 1][j].type != '1' && backgnd[i + 1][j].dis < backgnd[i][j].dis) {
        return DOWN;
    }
    if (j - 1 >= 0 && backgnd[i][j - 1].type != '1' && backgnd[i][j - 1].dis < backgnd[i][j].dis) {
        return LEFT;
    }
    if (i - 1 >= 0 && backgnd[i - 1][j].type != '1' && backgnd[i - 1][j].dis < backgnd[i][j].dis) {
        return UP;
    }

    return 'x';

}

void floor::BackHome(robot *Robot, cell **backgnd, int *i, int *j, queue<int> *row, queue<int> *col) {
    if ((*i) - 1 >= 0 && backgnd[(*i) - 1][*j].type != '1' && backgnd[(*i) - 1][*j].dis < backgnd[*i][*j].dis) {
        (*i)--;
        Robot->rem_life--;
        row->push(*i);
        col->push(*j);
    } else if ((*j) - 1 >= 0 && backgnd[*i][(*j) - 1].type != '1' && backgnd[*i][(*j) - 1].dis < backgnd[*i][*j].dis) {
        (*j)--;
        Robot->rem_life--;
        row->push(*i);
        col->push(*j);
    } else if ((*i) + 1 < size_r && backgnd[(*i) + 1][*j].type != '1' && backgnd[(*i) + 1][*j].dis < backgnd[*i][*j].dis) {
        (*i)++;
        Robot->rem_life--;
        row->push(*i);
        col->push(*j);
    } else {
        (*j)++;
        Robot->rem_life--;
        row->push(*i);
        col->push(*j);
    }
}