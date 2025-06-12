#include <iostream>
#include <fstream>
#include <list>
#include <string>

using namespace std;

class utilizator
{
protected:
	string adresa_mail;
	string parola;
	string nume;
	string prenume;
	string rol;
public:
	utilizator(string _adresa_mail, string _parola, string _nume, string _prenume, string _rol)
{
	this->adresa_mail = _adresa_mail;
	this->parola = _parola;
	this->nume = _nume;
	this->prenume = _prenume;
	this->rol = _rol;
}
	   virtual void afisare() = 0;
};

class user :public utilizator
{
	string varsta;
	string adresa;
public:
	user(string _adresa_mail, string _parola, string _nume, string _prenume, string _rol, string _varsta, string _adresa) :utilizator(_adresa_mail, _parola, _nume, _prenume,_rol)
	{
		this->varsta = _varsta;
		this->adresa = _adresa;
	}
	void afisare() {
		cout << "\nAdresa de mail : " << adresa_mail;
		cout << "\nParola : " << parola;
		cout << "\nNume : " << nume;
		cout << "\nPrenume : " << prenume;
		cout << "\nRol:" << rol;
		cout << "\nVarsta : " << varsta;
		cout << "\nAdresa : " << adresa;

	}
};
class manager :public utilizator
{
	string domeniul;
	string vechimea;
public:
    manager(string _adresa_mail, string _parola, string _nume, string _prenume, string _rol, string _domeniul, string _vechimea) :utilizator(_adresa_mail, _parola,_nume, _prenume,_rol)
{
	this->domeniul = _domeniul;
	this->vechimea = _vechimea;
}
		void afisare() {
			cout << "\nAdresa de mail : " << adresa_mail;
			cout << "\nParola : " << parola;
			cout << "\nNume : " << nume;
			cout << "\nPrenume : " << prenume;
			cout << "\nRol:" << rol;
			cout << "\nDomeniul : " << domeniul;
			cout << "\nVechimea : " << vechimea;

		}
};
void adaugaremanuala(list<utilizator*>& utilizatori)
{
	string adresa_mail;
	string parola;
	string nume;
	string prenume;
	string varsta;
	string adresa;
	string domeniul;
	string vechimea;
	string rol;
	cout << "adresa_mail:";
	cin >> adresa_mail;
	cout << "parola:";
	cin >> parola;
	cout << "nume:";
	cin >> nume;
	cout << "prenume:";
	cin >> prenume;
	cout << "rol = ";
	cin >> rol;
	if (rol == "1")
	{
		cout << "varsta:";
		cin >> varsta;
		cout << "adresa:";
		cin >> adresa;
		rol = "user";
		utilizator* u = new user(adresa_mail, parola, nume, prenume, rol, varsta, adresa);
		utilizatori.push_back(u);
	}
	else if (rol == "2")
	{
		cout << "domeniul : ";
		cin >> domeniul;
		cout << "vechimea : ";
		cin >> vechimea;
		rol = "manager";
		utilizator* u = new manager(adresa_mail, parola, nume, prenume, rol, domeniul, vechimea);
		utilizatori.push_back(u);
	}


}
void adaugare(list<utilizator*>& utilizatori)
{
	string adresa_mail;
	string parola;
	string nume;
	string prenume;
	string varsta;
	string adresa;
	string domeniul;
	string vechimea;
	string rol;
	ifstream fin("fisier.txt");
	while (!fin.eof())
	{
		fin >> adresa_mail;
		fin >> parola;
		fin >> nume;
		fin >> prenume;
		fin >> rol;
		if(rol=="1")
		{
			fin >> varsta;
			fin >> adresa;
			rol = "user";
			utilizator* u = new user(adresa_mail, parola, nume, prenume, rol, varsta, adresa);
			utilizatori.push_back(u);
		}
		else if (rol == "2")
		{
			fin >> domeniul;
			fin >> vechimea;
			rol = "manager";
			utilizator* u = new manager(adresa_mail, parola, nume, prenume, rol, domeniul, vechimea);
			utilizatori.push_back(u);
		}
	}

}

void afisare_lista(list<utilizator*>& utilizatori)
{
	for (utilizator* u : utilizatori)
	{
		cout << "\n----------------";
		u->afisare();
		cout << "\n----------------";
	}
}

int main(){
	list<utilizator*>utilizatori;
	int opt;
	do
	{
		cout << "\n-----------------------------";
		cout << "\0.Iesire";
		cout << "\n1.Adaugare in lista";
		cout << "\n2.Adaugare manuala in lista";
		cout << "\n3.Afisare lista";
		cout << "\n4.Stergere dupa mail";
		cout << "\n5.Modificare parola";
		cout << "\n6.Verificare utilizator multiplu";
		cout << "\nOptiunea dumneavoastra: ";
		cin >> opt;
		switch (opt)
		{
		case 1:adaugare(utilizatori);
			break;
		case 2:adaugaremanuala(utilizatori);
			break;
		case 3:afisare_lista(utilizatori);
			break;
		case 4:
			break;
		case 5:
			break;
		case 6:
			break;
		case 0:
			exit(0);
		default:exit(0);
			break;
		}
	} while (true);
	return 0;
};