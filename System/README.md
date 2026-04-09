
# Table of Contents
- [Usage](#usage)
- [Environment Configuration](#environment-configuration)
  - [Runtime environment](#runtime-environment)
  - [Development environment](#development-environment)
    - [Database Configuration](#database-configuration)
- [Deployment steps](#deployment-steps)
- [Directory Structure Description](#directory-structure-description)
- [Contributors](#contributors)

# Demo

<!-- Add a demo for your project -->

This system is an experimental platform for charging scheduling in Traffic Wireless Sensor Networks (TWSNs).
The charging scheduling process consists of two key steps: collection of roadside sensor node coordinates and calculation of optimal charging scheduling plans.
The system provides four test datasets for evaluation. You may also use the mark mode to manually label node coordinates.



# Usage
If you only need to run this project for TWSN charging scheduling, then only need to configure [runtime environment](#runtime-environment). After downloading, execute the following code to deploy the gateway by accessing `localhost:8082`.
```sh
java xxx.jar -ak xxxxx
```

This project supports calling python algorithm for calculation, but needs to configure the [python runtime environment]().


# Environment Configuration
## Runtime environment
* JRE 1.8
* python 3.8+


## Development environment
If you need to modify the back-end code or modify the front-end page, you need to configure the following environment
* Java Version: Temurin JDK 1.8
* Node.js Version：v16.15.1
* npm Version：9.6.5
* vite Version: 4.4.4

### Database Configuration
To configure the database for the project, modify the `application.properties` file within the `src/main/resources` directory with the following details:
```properties
spring.datasource.url=jdbc:mysql://<db-host>:<port>/<database-name>
spring.datasource.username=<db-username>
spring.datasource.password=<db-password>
```
Replace \<db-host>, \<port>, \<database-name>, \<db-username>, and \<db-password> with your actual database settings. Ensure that your MySQL server is running and accessible at the provided URL and port. The database should be set up with the necessary tables and permissions for the user specified.

The code currently uses the `Marks2` database in the local database by default, with the following account and password configuration:
```properties
spring.datasource.url=jdbc:mysql://${DB_HOST:localhost}:3306/Marks2?serverTimezone=UTC&useUnicode=true&characterEncoding=utf8&useSSL=false&allowPublicKeyRetrieval=true
spring.datasource.username=${DB_USERNAME:road_data}
spring.datasource.password=${DB_PASSWORD:123456}
```
You can create a new database locally according to the above configuration, and initialize the tables in the database through the `road_data.sql` file to complete the database configuration.

# Deployment steps

Clone this repository and navigate inside the project root folder.

If you want to modify the style of the front end, please enter the project root directory, and then execute the following code.
```sh
cd ./vue
npm install
```
After installing the dependencies, you can modify the front-end related code in `system/vue/src/App.vue`, and then execute the following command to view the front-end interface.
```sh
npm run dev
```

# Directory Structure Description
```sh
|-- README.md     // help
|-- src           // Backend processing related
|-- vue           // Front-end vue code
|-- mvnw
|-- mvnw.cmd
|-- pom.xml
```

# Contributors

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/drggboy"><img src="https://avatars.githubusercontent.com/u/47265146?v=4?s=100" width="100px;" alt="drggboy"/><br /><sub><b>drggboy</b></sub></a><br /><a href="https://github.com/grtm77/mytest_v2/commits?author=drggboy" title="Code">💻</a> <a href="https://github.com/grtm77/mytest_v2/commits?author=drggboy" title="Documentation">📖</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/grtm77"><img src="https://avatars.githubusercontent.com/u/50659884?v=4?s=100" width="100px;" alt="grtm77"/><br /><sub><b>grtm77</b></sub></a><br /><a href="https://github.com/grtm77/mytest_v2/commits?author=grtm77" title="Code">💻</a> <a href="https://github.com/grtm77/mytest_v2/commits?author=grtm77" title="Documentation">📖</a></td>
    </tr>
  </tbody>
  <tfoot>
    <tr>
      <td align="center" size="13px" colspan="7">
        <img src="https://raw.githubusercontent.com/all-contributors/all-contributors-cli/1b8533af435da9854653492b1327a23a4dbd0a10/assets/logo-small.svg">
          <a href="https://all-contributors.js.org/docs/en/bot/usage">Add your contributions</a>
        </img>
      </td>
    </tr>
  </tfoot>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!






