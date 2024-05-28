#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <ctime>
#include <limits>
using namespace std;

const int BOARD_SIZE = 38;
const int START_BALANCE = 5000;

// Player class definition
class Player {
private:
    string name;
    char gender;
    int balance;
    int position;

public:
    // Constructor to initialize a player
    Player(string playerName = "Computer", char playerGender = 'N', int playerBalance = START_BALANCE, int playerPosition = 0)
        : name(playerName), gender(playerGender), balance(playerBalance), position(playerPosition) {}

    // Method to set up player information
    void setupPlayer() {
        cout << "Enter player name: ";
        cin >> name;
        cout << "Enter player gender (M/F): ";
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        while(!(cin >> gender)|| gender != 'M' && gender != 'F'){
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << "Invalid input. Please enter M or F: ";

        }
        balance = START_BALANCE;
        position = 0;
    }

    // Method to save player information to a file
    void savePlayerInfo() const {
        ofstream file("players.txt", ios::app);
        if (file.is_open()) {
            file << name << " " << gender << " " << balance << " " << position << endl;
            file.close();
        } else {
            cout << "Unable to open file for writing!" << endl;
        }
    }

    // Method to update player balance and position in the file
    void updateBalanceAndPositionInFile() const {
        ofstream file("players.txt", ios::app);
        if (file.is_open()) {
            file << name << " " << gender << " " << balance << " " << position << endl;
            file.close();
        } else {
            cout << "Unable to open file for writing!" << endl;
        }
    }

    // Getter for position
    int getPosition() const {
        return position;
    }

    // Getter for balance
    int getBalance() const {
        return balance;
    }

    // Method to move the player based on dice value
    void move(int diceValue, int prices[], bool clockwise) {
        if (clockwise) {
            position = (position + diceValue) % BOARD_SIZE;
        } else {
            position = (position - diceValue + BOARD_SIZE) % BOARD_SIZE;
        }
        
        if (position == 0) {
            cout << name << " landed on GO" << endl;
            // Add any bonus or special action here if needed
        } else if (position == BOARD_SIZE - 1) {
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

    // Getter for name
    string getName() const {
        return name;
    }
};

// Game class definition
class Game {
private:
    Player player;
    Player computer;
    int prices[BOARD_SIZE];

    // Method to roll a dice
    int rollDice() const {
        return rand() % 6 + 1;
    }

public:
    // Constructor to initialize the game
    Game() : computer("Computer") {
        srand(time(0));
        for (int i = 0; i < BOARD_SIZE; ++i) {
            prices[i] = rand() % 291 + 10; // Random price between 10 and 300
        }
        prices[0] = 0; // "GO" square
        prices[BOARD_SIZE - 1] = 0; // "JAIL" square
    }

    // Method to clear player file at the start of the game
    void clearPlayerFile() const {
        ofstream file("players.txt", ios::trunc);
        if (file.is_open()) {
            file.close();
        } else {
            cout << "Unable to open file for writing!" << endl;
        }
    }

    // Method to set up the game
    void setup() {
        clearPlayerFile();
        player.setupPlayer();
        player.savePlayerInfo();
        computer.savePlayerInfo();
    }

    // Method to play the game
    void play() {
        while (player.getBalance() > 0 && computer.getBalance() > 0) {
            string choice;
            cout << "Do you want to roll the dice? (yes/no): ";
            cin >> choice;
            if (choice == "no") {
                cout << player.getName() << " has chosen to quit the game." << endl;
                break;
            } else if (choice == "yes") {
                int diceValue = rollDice();
                cout << player.getName() << " rolled a " << diceValue << " (clockwise)";
                player.move(diceValue, prices, true);
                cout << " (" << player.getName() << " is now at position " << player.getPosition() << " with balance " << player.getBalance() << ')' << endl;
                player.updateBalanceAndPositionInFile();

                if (player.getBalance() <= 0) break;

                diceValue = rollDice();
                cout << computer.getName() << " rolled a " << diceValue << " (anticlockwise)";
                computer.move(diceValue, prices, false);
                cout << " (" << computer.getName() << " is now at position " << computer.getPosition() << " with balance " << computer.getBalance() << ')' << endl;
                computer.updateBalanceAndPositionInFile();

                if (computer.getBalance() <= 0) break;
            } else {
                cout << "Invalid choice. Please enter 'yes' or 'no'." << endl;
            }
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

// Main function
int main() {
    Game game;
    game.setup();
    game.play();

    return 0;
}
