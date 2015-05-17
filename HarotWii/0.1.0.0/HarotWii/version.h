#define FC_VERSION_MAJOR            1  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            8  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      0  // increment when a bug is fixed

#define GIT_SHORT_REVISION_LENGTH   7 // lower case hexadecimal digits.
extern char* shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern char* buildDate;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
extern char* buildTime;  // "HH:MM:SS"