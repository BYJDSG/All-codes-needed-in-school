#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <ctime>

using namespace std;

const int BOARD_SIZE = 38;
const int START_BALANCE = 5000;

class Player {
private:
    string name;
    char gender;
    int balance;
    int position;

public:
    Player(string playerName = "Computer", char playerGender = 'N', int playerBalance = START_BALANCE, int playerPosition = 0)
        : name(playerName), gender(playerGender), balance(playerBalance), position(playerPosition) {}

    void setupPlayer() {
        cout << "Enter player name: ";
        cin >> name;
        cout << "Enter player gender (M/F): ";
        cin >> gender;
        balance = START_BALANCE;
        position = 0;
    }

    void savePlayerInfo() const {
        ofstream file("players.txt", ios::app);
        if (file.is_open()) {
            file << name << " " << gender << " " << balance << endl;
            file.close();
        } else {
            cout << "Unable to open file for writing!" << endl;
        }
    }

    void updateBalanceInFile() const {
        ofstream file("players.txt", ios::app);
        if (file.is_open()) {
            file << name << " " << gender << " " << balance << endl;
            file.close();
        } else {
            cout << "Unable to open file for writing!" << endl;
        }
    }

    int getPosition() const {
        return position;
    }

    int getBalance() const {
        return balance;
    }

    void move(int diceValue, int prices[]) {
        position = (position + diceValue) % BOARD_SIZE;
        if (position == BOARD_SIZE - 1) {
            cout << name << " landed on JAIL and loses the game!" << endl;
            balance = -1;
        } else {
            if (balance >= prices[position]) {
                balance -= prices[position];
            } else {
                cout << name << " does not have enough balance to pay the price and loses the game!" << endl;
                balance = -1;
            }
        }
    }

    string getName() const {
        return name;
    }
};

class Game {
private:
    Player player;
    Player computer;
    int prices[BOARD_SIZE];

    int rollDice() const {
        return rand() % 6 + 1;
    }

public:
    Game() : computer("Computer") {
        srand(time(0));
        for (int i = 0; i < BOARD_SIZE; ++i) {
            prices[i] = rand() % 291 + 10; // Random price between 10 and 300
        }
        prices[0] = 0; // "GO" square
        prices[BOARD_SIZE - 1] = 0; // "JAIL" square
    }

    void clearPlayerFile() const {
        ofstream file("players.txt", ios::trunc);
        if (file.is_open()) {
            file.close();
        } else {
            cout << "Unable to open file for writing!" << endl;
        }
    }

    void setup() {
        clearPlayerFile();
        player.setupPlayer();
        player.savePlayerInfo();
        computer.savePlayerInfo();
    }

    void play() {
        while (player.getBalance() > 0 && computer.getBalance() > 0) {
            string choice;
            cout << "Do you want to roll the dice? (yes/no): ";
            cin >> choice;
            if (choice != "yes") {
                cout << player.getName() << " has chosen to quit the game." << endl;
                break;
            }

            int diceValue = rollDice();
            cout << player.getName() << " rolled a " << diceValue << endl;
            player.move(diceValue, prices);
            player.updateBalanceInFile();

            if (player.getBalance() <= 0) break;

            diceValue = rollDice();
            cout << computer.getName() << " rolled a " << diceValue << endl;
            computer.move(diceValue, prices);
            computer.updateBalanceInFile();

            if (computer.getBalance() <= 0) break;
        }

        if (player.getBalance() <= 0) {
            cout << "Computer wins with a balance of " << computer.getBalance() << endl;
        } else if (computer.getBalance() <= 0) {
            cout << player.getName() << " wins with a balance of " << player.getBalance() << endl;
        } else {
            cout << player.getName() << " quit the game with a balance of " << player.getBalance() << endl;
        }
    }
};

int main() {
    Game game;
    game.setup();
    game.play();

    return 0;
}
