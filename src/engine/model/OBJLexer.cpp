#include "OBJLexer.h"

#include <limits>

using namespace std;

OBJLexer::OBJLexer(const char *path) :
	m_token(TOK_EOL),
	m_line(0),
	m_value(0),
	m_error(false)
{
	m_file.open(path);
	if (!m_file.is_open()) {
		m_str.reserve(40);
		snprintf((char*)m_str.data(), m_str.capacity(), "Unable to open file: %s", path);
		m_error = true;
	}
}

OBJLexer::OBJLexer(const string &path) :
	OBJLexer(path.c_str())
{
}

OBJLexer::~OBJLexer()
{
	if (m_file.is_open())
		m_file.close();
}

void OBJLexer::SkipWS(void)
{
	char c;
	while (true) {
		c = m_file.get();

		if (isspace(c) && (c != '\n'))
			continue;
		if (c == '#') {
			m_file.ignore(numeric_limits<streamsize>::max(), '\n');
			continue;
		}
		m_file.unget();
		break;
	}
}

bool OBJLexer::ProcessNext(void)
{
	char c;
	string tmp;

	if (m_error) return false;
	if (m_file.eof())
		return false;

	// skip whitespaces and comments
	SkipWS();

	c = m_file.get();

	if (c == EOF)
		return false;

	if (c == '\n') {
		m_line++;
		m_token = TOK_EOL;
		return true;
	}
	if (c == '/') {
		m_token = TOK_SLASH;
		return true;
	}
	if (c == '.' || c == '-' || c == '+' || isdigit(c)) {
		m_file.unget();
		m_file >> m_value;
		m_token = TOK_NUMBER;
		return true;
	}
	if (isgraph(c)) {
		m_file.unget();
		m_file >> m_str;
		m_token = TOK_STRING;
		return true;
	}

	m_str.reserve(40);
	snprintf((char*)m_str.data(), m_str.capacity(), "Unrecognized char: '%c' [dec: %d], line %d", c, c, m_line);
	m_error = true;
	return false;
}


